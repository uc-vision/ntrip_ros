#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import traceback
from rtcm_msgs.msg import Message

from base64 import b64encode

from http.client import HTTPConnection
from .ntrip_response import NTRIPResponse
from dataclasses import dataclass
from threading import Thread, Event
from rcl_interfaces.msg import SetParametersResult


import time
from rclpy.impl.rcutils_logger import RcutilsLogger


''' This is to fix the IncompleteRead error
    http://bobrochel.blogspot.com/2010/11/bad-servers-chunked-encoding-and.html'''
import http.client
def patch_http_response_read(func):
    def inner(*args):
        try:
            return func(*args)
        except http.client.IncompleteRead as e:
            return e.partial
    return inner
http.client.HTTPResponse.read = patch_http_response_read(http.client.HTTPResponse.read)

@dataclass
class NTripConfig:
  ntrip_server: str
  ntrip_user: str
  ntrip_pass: str
  ntrip_stream: str
  nmea_gga: str


class NTripClient:

    def __init__(self, rtcm_publisher, config: NTripConfig, condition: Event, retry_time: float = 15.0):
        self.ntrip_server = config.ntrip_server
        self.ntrip_user = config.ntrip_user
        self.ntrip_pass = config.ntrip_pass
        self.ntrip_stream = config.ntrip_stream
        self.nmea_gga = config.nmea_gga

        self.pub = rtcm_publisher
        self.condition = condition
        self.retry_time = retry_time
        self.logger = RcutilsLogger(name="Ntrip_client_logger")


    def run(self):
      try:
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode((self.ntrip_user + ':' + str(self.ntrip_pass)).encode("utf-8")).decode("utf-8")
        }
        connection = HTTPConnection(self.ntrip_server)
        connection.request('GET', '/'+self.ntrip_stream, self.nmea_gga, headers)
        # Patch the response class to work with ublox ntrip server.
        connection.response_class = NTRIPResponse
        response = connection.getresponse()
        if response.status != 200: raise Exception("Unexpected http response code: {response.status}")
        buf = bytes()
        rmsg = Message()
        restart_count = 0

        while not self.condition.is_set():
            ''' This now separates individual RTCM messages and publishes each one on the same topic '''
            data = response.read(1)
            if len(data) != 0:
                if data[0] == 211:
                    buf += data
                    data = response.read(2)
                    buf += data
                    cnt = data[0] * 256 + data[1]
                    data = response.read(2)
                    buf += data
                    typ = (data[0] * 256 + data[1]) // 16
                    #print (str(datetime.now()), cnt, typ)
                    cnt = cnt + 1
                    for x in range(cnt):
                        data = response.read(1)
                        buf += data
                    rmsg.message = buf
                    rmsg.header.stamp = rclpy.time.Time().to_msg()
                    self.pub.publish(rmsg)
                    buf = bytes()
                else: 
                    #print (data)
                    pass
            else:
                ''' If zero length data, close connection and reopen it '''
                restart_count = restart_count + 1
                self.logger.info(f'Zero length {restart_count}')
                connection.close()
                time.sleep(self.retry_time)   # you get banned from rtk2go for rapid retries
                connection = HTTPConnection(self.ntrip_server)
                connection.request('GET', '/'+self.ntrip_stream, self.nmea_gga, headers)
                response = connection.getresponse()
                if response.status != 200: raise Exception("Unexcpted http resopnse code: {response.status}")
                buf = ""

        connection.close()
      except Exception as e:
        self.logger.error(traceback.format_exc())  
        return

         

class Ntrip(Node):
    
    def declare_server_parameters(self, hosts):
      for name in hosts:
        self.declare_parameter(name + '.host', '')
        self.declare_parameter(name + '.user', '')
        self.declare_parameter(name + '.pass', '')
        self.declare_parameter(name + '.stream', '')
        self.declare_parameter(name + '.nmea_gga', '')
    
    def parameters_callback(self, params):
      for param in params:
        if param.name == "server":
          if param.value not in self.servers:
            self.get_logger().warn(f'{param.value} not in servers')
            continue
          self.parameter_change = True
          self.server = param.value
      return SetParametersResult(successful=True)

    def __init__(self):
      super().__init__('ntripclient')

      self.srv_thread = None
      self.condition = None
      self.parameter_change = False

      self.declare_parameter('rtcm_topic', 'rtcm')
      self.declare_parameter('servers', [''])
      self.declare_parameter('server', '')
      self.declare_parameter('retry_time_sec', 20.0)
      
      self.rtcm_topic = self.get_parameter('rtcm_topic').value
      self.server = self.get_parameter('server').value
      self.servers = self.get_parameter('servers').value

      self.declare_server_parameters(self.servers)
      self.add_on_set_parameters_callback(self.parameters_callback)

      self.get_logger().info(f'Start Ntrip Server: {self.server}')
      self.start_ntrip_thread()

      self.retry_time_sec = self.get_parameter('retry_time_sec').value
      self.last_restart_time = self.get_clock().now()
      self.timer = self.create_timer(1 / 10.0, self.loop)

    def get_ntrip_client(self):
      config = NTripConfig(
        ntrip_server = self.get_parameter(self.server + '.host').value,
        ntrip_user = self.get_parameter(self.server +'.user').value,
        ntrip_pass = self.get_parameter(self.server +'.pass').value,
        ntrip_stream = self.get_parameter(self.server +'.stream').value,
        nmea_gga = self.get_parameter(self.server +'.nmea_gga').value
      )
      self.condition = Event()
      pub = self.create_publisher(Message, self.rtcm_topic, 10)
      return NTripClient(pub, config, self.condition, self.retry_time_sec)
    
    def is_thread_alive(self):
      if self.srv_thread is not None:
        return self.srv_thread.is_alive()
      
    def restart_ntrip_thread(self):
      if self.srv_thread is not None:
        self.get_logger().info('Stopping Ntrip Server')
        self.stop_ntrip_thread()

      self.get_logger().info(f'Start Ntrip Server: {self.server}')
      self.start_ntrip_thread()
    
    def start_ntrip_thread(self):
      if self.srv_thread is None:
        ntrip_client = self.get_ntrip_client()
        
        self.srv_thread = Thread(target=ntrip_client.run, daemon=True)
        self.srv_thread.start()

    def stop_ntrip_thread(self):
      self.condition.set()
      self.srv_thread.join()
      self.srv_thread = None

    def destroy_node(self):
      self.stop_ntrip_thread()
      super().destroy_node()

    def loop(self):
      if self.parameter_change and self.is_thread_alive():
        self.restart_ntrip_thread()
        self.parameter_change = False

      current_time = self.get_clock().now()
      sec_since_last_command = ( current_time - self.last_restart_time ).nanoseconds / 1e9
      if not self.is_thread_alive() and sec_since_last_command > self.retry_time_sec:
        self.last_restart_time = current_time
        self.get_logger().info('Dead. Restarting...')
        self.restart_ntrip_thread()
        

def main(args=None):
  rclpy.init(args=args)
  node = Ntrip()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    logger = RcutilsLogger(name="ntrip_client")
    logger.info('Keyboard interrupt. Shutting down...')
  finally:
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
  main()

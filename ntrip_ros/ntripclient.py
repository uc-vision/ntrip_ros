#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

#from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message

from base64 import b64encode

from http.client import HTTPConnection
from .ntrip_response import NTRIPResponse

import time

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



class NTripClient(Node):
    def __init__(self):
        super().__init__('ntripclient')

        self.declare_parameters('', [
            ('rtcm_topic', 'rtcm'),
            ('nmea_topic', 'nmea'),
            ('ntrip_server', Parameter.Type.STRING),
            ('ntrip_user', Parameter.Type.STRING),
            ('ntrip_pass', Parameter.Type.STRING),
            ('ntrip_stream', Parameter.Type.STRING),
            ('nmea_gga', Parameter.Type.STRING)
            ])
        self.rtcm_topic = self.get_parameter('rtcm_topic').value
        self.nmea_topic = self.get_parameter('nmea_topic').value

        self.ntrip_server = self.get_parameter('ntrip_server').value
        self.ntrip_user = self.get_parameter('ntrip_user').value
        self.ntrip_pass = self.get_parameter('ntrip_pass').value
        self.ntrip_stream = self.get_parameter('ntrip_stream').value
        self.nmea_gga = self.get_parameter('nmea_gga').value

        self.pub = self.create_publisher(Message, self.rtcm_topic, 10)

    def run(self):

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
        while(rclpy.ok()):
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
                self.get_logger().info(f'Zero length {restart_count}')
                connection.close()
                time.sleep(15)   # you get banned from rtk2go for rapid retries
                connection = HTTPConnection(self.ntrip_server)
                connection.request('GET', '/'+self.ntrip_stream, self.nmea_gga, headers)
                response = connection.getresponse()
                if response.status != 200: raise Exception("Unexcpted http resopnse code: {response.status}")
                buf = ""

        connection.close()

def main(args=None):
  rclpy.init(args=args)
  node = NTripClient()
  node.get_logger().info('Initialising client...')
  time.sleep(10)
  node.get_logger().info('Initialised')
  node.run()
  node.get_logger().info('Destroying client...')
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

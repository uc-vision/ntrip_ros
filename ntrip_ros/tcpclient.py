#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from rtcm_msgs.msg import Message

import socket
import time


class TCPClient(Node):
    def __init__(self):
        super().__init__('ntripclient')

        self.declare_parameter('rtcm_topic', 'rtcm')
        self._rtcm_topic = self.get_parameter('rtcm_topic').value

        self.pub = self.create_publisher(Message, self._rtcm_topic, 10)

    def monitor(self):
        while(rclpy.ok()):
            self.run()
            time.sleep(10)


    def run(self):
        connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connection.connect(("rtcm.autonabit.nz", 5016))
        buf = bytes()
        rmsg = Message()
        while(rclpy.ok()):

            ''' This now separates individual RTCM messages and publishes each one on the same topic '''
            data = connection.recv(1)
            if len(data) != 0:
                if data[0] == 211:
                    buf += data
                    data = connection.recv(2)
                    buf += data
                    cnt = data[0] * 256 + data[1]
                    data = connection.recv(2)
                    buf += data
                    typ = (data[0] * 256 + data[1]) // 16
                    #print (str(datetime.now()), cnt, typ)
                    cnt = cnt + 1
                    for x in range(cnt):
                        data = connection.recv(1)
                        buf += data
                    rmsg.message = buf
                    rmsg.header.seq += 1
                    rmsg.header.stamp = rclpy.time.Time()
                    self.pub.publish(rmsg)
                    buf = bytes()
                else: 
                    #print (data)
                    pass
            else:
                ''' If zero length data, close connection and reopen it '''
                self.get_logger().info('Zero length')
                return 

        connection.close()

def main(args=None):
  rclpy.init(args=args)
  node = TCPClient()
  time.sleep(10)
  node.monitor()
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
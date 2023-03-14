#!/usr/bin/env python

import rospy
from datetime import datetime

from rtcm_msgs.msg import Message

from base64 import b64encode
import socket

import time


class tcpclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic', 'rtcm')

        self.pub = rospy.Publisher(self.rtcm_topic, Message, queue_size=10)

    def monitor(self):
        while(not rospy.is_shutdown()):
            self.run()
            time.sleep(10)


    def run(self):
        connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connection.connect(("rtcm.autonabit.nz", 5016))
        buf = bytes()
        rmsg = Message()
        while(not rospy.is_shutdown()):

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
                    rmsg.header.stamp = rospy.get_rostime()
                    self.pub.publish(rmsg)
                    buf = bytes()
                else: 
                    #print (data)
                    pass
            else:
                ''' If zero length data, close connection and reopen it '''
                rospy.logwarn(f"Zero length")
                return 

        connection.close()

if __name__ == '__main__':
    c = tcpclient()
    c.run()


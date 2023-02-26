#!/usr/bin/env python

import rospy
from datetime import datetime

#from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message

from base64 import b64encode
from threading import Thread

from http.client import HTTPConnection
from http.client import IncompleteRead
from ntrip_response import NTRIPResponse

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



class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic', 'rtcm')
        self.nmea_topic = rospy.get_param('~nmea_topic', 'nmea')

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')
        self.nmea_gga = rospy.get_param('~nmea_gga')

        self.pub = rospy.Publisher(self.rtcm_topic, Message, queue_size=10)

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
        if response.status != 200: raise Exception("Unexcpted http resopnse code: {response.status}")
        buf = bytes()
        rmsg = Message()
        restart_count = 0
        while(not rospy.is_shutdown()):

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
                    rmsg.header.seq += 1
                    rmsg.header.stamp = rospy.get_rostime()
                    self.pub.publish(rmsg)
                    buf = bytes()
                else: 
                    #print (data)
                    pass
            else:
                ''' If zero length data, close connection and reopen it '''
                restart_count = restart_count + 1
                rospy.logwarn(f"Zero length {restart_count}")
                connection.close()
                connection = HTTPConnection(self.ntrip_server)
                connection.request('GET', '/'+self.ntrip_stream, self.nmea_gga, headers)
                response = connection.getresponse()
                if response.status != 200: raise Exception("Unexcpted http resopnse code: {response.status}")
                buf = ""

        connection.close()

if __name__ == '__main__':
    c = ntripclient()
    c.run()


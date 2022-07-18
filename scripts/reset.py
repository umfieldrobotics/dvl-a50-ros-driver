#!/bin/env python

import socket
import sys
from time import sleep
import json
import rospy


def connect():
    global s, TCP_IP, TCP_PORT
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_IP, TCP_PORT))
        s.settimeout(1)
    except socket.error as err:
        rospy.logerr("No route to host, DVL might be booting? {}".format(err))
        sleep(1)
        connect()


def write_reset():
    global s
    reset_msg = {
        "command": "reset_dead_reckoning"
    }
    res = s.send(json.dumps(reset_msg).encode('utf-8')) 
    print(res)
#!/bin/env python3

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


if __name__ == '__main__':
    global s, TCP_IP, TCP_PORT, do_log_raw_data
    rospy.init_node('a50_pub', anonymous=False)
    TCP_IP = rospy.get_param("~ip", "192.168.2.95")
    TCP_PORT = rospy.get_param("~port", 16171)
    do_log_raw_data = rospy.get_param("~do_log_raw_data", False)
    connect()
    write_reset()
    s.close()

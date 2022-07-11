#!/usr/bin/env python
import json
import socket
from time import sleep

import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import (PoseWithCovarianceStamped, TransformStamped,
                               TwistStamped, TwistWithCovarianceStamped)
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64, String
from waterlinked_a50_ros_driver.msg import DVL, DVLBeam


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


oldJson = ""

theDVL = DVL()
beam0 = DVLBeam()
beam1 = DVLBeam()
beam2 = DVLBeam()
beam3 = DVLBeam()


def getData():
    global oldJson, s
    raw_data = ""

    while not '\n' in raw_data:
        try:
            rec = s.recv(1)  # Add timeout for that
            if len(rec) == 0:
                rospy.logerr("Socket closed by the DVL, reopening")
                connect()
                continue
        except socket.timeout as err:
            rospy.logerr(
                "Lost connection with the DVL, reinitiating the connection: {}".format(err))
            connect()
            continue
        # raw_data = raw_data + rec
        raw_data = raw_data + rec.decode('utf-8')
    raw_data = oldJson + raw_data
    oldJson = ""
    raw_data = raw_data.split('\n')
    oldJson = raw_data[1]
    raw_data = raw_data[0]
    return raw_data


def publisher():
    pub_raw = rospy.Publisher('dvl/json_data', String, queue_size=10)
    pub = rospy.Publisher('dvl/data', DVL, queue_size=10)
    twist_cov_pub = rospy.Publisher(
        'dvl/twist_cov', TwistWithCovarianceStamped, queue_size=10)
    twist_pub = rospy.Publisher(
        'dvl/twist', TwistStamped, queue_size=10)
    pose_pub = rospy.Publisher(
        'dvl/vel_coverage', PoseWithCovarianceStamped, queue_size=10)
    pub_quat = rospy.Publisher(
        'dvl/local_position', PoseWithCovarianceStamped, queue_size=10)
    vx_pub = rospy.Publisher(
        'plotter/vx', Float64, queue_size=10)
    vy_pub = rospy.Publisher(
        'plotter/vy', Float64, queue_size=10)
    vz_pub = rospy.Publisher(
        'plotter/vz', Float64, queue_size=10)
    altitude_pub = rospy.Publisher(
        'plotter/dvl_altitude', Float64, queue_size=10)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():

        # # Publish tf data
        # br = tf.TransformBroadcaster()
        # tf_msg = TransformStamped()
        # tf_msg.header.stamp = rospy.Time.now()
        # tf_msg.header.frame_id = "base_link"
        # tf_msg.child_frame_id = "dvl_link"
        # tf_msg.transform.translation.x = 0.042
        # tf_msg.transform.translation.y = 0.0
        # tf_msg.transform.translation.z = -0.21062
        # rpy = [np.pi, 0, -np.pi / 2]
        # rot = R.from_euler('xyz', rpy)
        # qx = rot.as_quat()[0]
        # qy = rot.as_quat()[1]
        # qz = rot.as_quat()[2]
        # qw = rot.as_quat()[3]
        # tf_msg.transform.rotation.x = qx
        # tf_msg.transform.rotation.y = qy
        # tf_msg.transform.rotation.z = qz
        # tf_msg.transform.rotation.w = qw
        # br.sendTransform(tf_msg)

        raw_data = getData()
        if do_log_raw_data:
            # rospy.loginfo(raw_data)
            pub_raw.publish(raw_data)

        data = json.loads(raw_data)
        if data["type"] == "velocity":
            # Populate the message for DVL
            theDVL.header.stamp = rospy.Time.now()
            theDVL.header.frame_id = "dvl_link"
            theDVL.time = data["time"]
            theDVL.velocity.x = data["vx"]
            theDVL.velocity.y = data["vy"]
            theDVL.velocity.z = data["vz"]

            cov = np.array(data["covariance"]).reshape(1, -1)[0].tolist()
            theDVL.velocity_covariance = cov

            theDVL.fom = data["fom"]
            theDVL.altitude = data["altitude"]
            altitude_pub.publish(theDVL.altitude)
            theDVL.velocity_valid = data["velocity_valid"]
            theDVL.status = data["status"]
            theDVL.form = data["format"]

            beam0.id = data["transducers"][0]["id"]
            beam0.velocity = data["transducers"][0]["velocity"]
            beam0.distance = data["transducers"][0]["distance"]
            beam0.rssi = data["transducers"][0]["rssi"]
            beam0.nsd = data["transducers"][0]["nsd"]
            beam0.valid = data["transducers"][0]["beam_valid"]

            beam1.id = data["transducers"][1]["id"]
            beam1.velocity = data["transducers"][1]["velocity"]
            beam1.distance = data["transducers"][1]["distance"]
            beam1.rssi = data["transducers"][1]["rssi"]
            beam1.nsd = data["transducers"][1]["nsd"]
            beam1.valid = data["transducers"][1]["beam_valid"]

            beam2.id = data["transducers"][2]["id"]
            beam2.velocity = data["transducers"][2]["velocity"]
            beam2.distance = data["transducers"][2]["distance"]
            beam2.rssi = data["transducers"][2]["rssi"]
            beam2.nsd = data["transducers"][2]["nsd"]
            beam2.valid = data["transducers"][2]["beam_valid"]

            beam3.id = data["transducers"][3]["id"]
            beam3.velocity = data["transducers"][3]["velocity"]
            beam3.distance = data["transducers"][3]["distance"]
            beam3.rssi = data["transducers"][3]["rssi"]
            beam3.nsd = data["transducers"][3]["nsd"]
            beam3.valid = data["transducers"][3]["beam_valid"]

            theDVL.beams = [beam0, beam1, beam2, beam3]

            vx_pub.publish(theDVL.velocity.x)
            vy_pub.publish(theDVL.velocity.y)
            vz_pub.publish(theDVL.velocity.z)

            pub.publish(theDVL)

            # Populate the message for twist cov
            twist_cov = TwistWithCovarianceStamped()
            twist_cov.header.stamp = rospy.Time.now()
            twist_cov.header.frame_id = "dvl_link"
            twist_cov.twist.twist.linear.x = data["vx"]
            twist_cov.twist.twist.linear.y = data["vy"]
            twist_cov.twist.twist.linear.z = data["vz"]
            tmp_cov = np.eye(6).reshape(-1)
            tmp_cov[:9] = cov
            twist_cov.twist.covariance = tmp_cov
            twist_cov_pub.publish(twist_cov)

            # Populate the message for twist
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.header.frame_id = "dvl_link"
            twist.twist.linear.x = data["vx"]
            twist.twist.linear.y = data["vy"]
            twist.twist.linear.z = data["vz"]
            twist_pub.publish(twist)

            # Purpose of visualization with RViz, which doesn't have the twist covariance message, puslish it as a pose
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "dvl_link"
            pose.pose.pose.position.x = data["vx"]
            pose.pose.pose.position.y = data["vy"]
            pose.pose.pose.position.z = data["vz"]
            pose.pose.covariance = tmp_cov
            pose_pub.publish(pose)

        elif data["type"] == "position_local":
            roll = data["roll"]
            pitch = data["pitch"]
            yaw = data["yaw"]
            std = data["std"]
            rot = R.from_euler('xyz', [roll, pitch, yaw])
            qx = rot.as_quat()[0]
            qy = rot.as_quat()[1]
            qz = rot.as_quat()[2]
            qw = rot.as_quat()[3]

            x = data['x']
            y = data['y']
            z = data['z']

            dvl_position = PoseWithCovarianceStamped()
            dvl_position.header.stamp = rospy.Time.now()
            dvl_position.header.frame_id = "dvl_link"
            dvl_position.pose.pose.position.x = x
            dvl_position.pose.pose.position.y = y
            dvl_position.pose.pose.position.z = z
            dvl_position.pose.pose.orientation.x = qx
            dvl_position.pose.pose.orientation.y = qy
            dvl_position.pose.pose.orientation.z = qz
            dvl_position.pose.pose.orientation.w = qw
            cov = std ** 2 * np.eye(6)
            dvl_position.pose.covariance = cov.reshape(-1,)

            pub_quat.publish(dvl_position)

        rate.sleep()


if __name__ == '__main__':
    global s, TCP_IP, TCP_PORT, do_log_raw_data
    rospy.init_node('a50_pub', anonymous=False)
    TCP_IP = rospy.get_param("~ip", "192.168.2.95")
    TCP_PORT = rospy.get_param("~port", 16171)
    do_log_raw_data = rospy.get_param("~do_log_raw_data", False)
    connect()
    try:
        publisher()
    except rospy.ROSInterruptException:
        s.close()

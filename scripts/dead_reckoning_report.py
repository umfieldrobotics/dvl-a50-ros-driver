#!/usr/bin/env python
import rospy
from scipy.spatial.transform import Rotation as R
from waterlinked_a50_ros_driver.msg import DVL, DVLBeam
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovariance
import json
import rosbag
import matplotlib.pyplot as plt


def main():
    rospy.init_node('dead_reckoning_report')
    bag = rospy.get_param('~bag', '')
    x, y, z = [], [], []
    roll, pitch, yaw = [], [], []
    pub_roll, pub_pitch, pub_yaw = [], [], []

    with rosbag.Bag(bag) as bag:
        for topic, msg, t in bag.read_messages(topics=['/dvl/json_data', '/dvl/local_position']):
            if topic == '/dvl/json_data':
                data = json.loads(msg.data)
                if 'x' in data.keys():
                    x.append(data['x'])
                    y.append(data['y'])
                    z.append(data['z'])
                    roll.append(data['roll'])
                    pitch.append(data['pitch'])
                    yaw.append(data['yaw'])
            if topic == '/dvl/local_position':
                qx = msg.pose.pose.orientation.x
                qy = msg.pose.pose.orientation.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                q = R.from_quat([qx, qy, qz, qw])
                roll_, pitch_, yaw_ = q.as_euler('xyz')
                pub_roll.append(roll_)
                pub_pitch.append(pitch_)
                pub_yaw.append(yaw_)

    fig, axs = plt.subplots(3, 2, figsize=(10, 10))
    gs = axs[0, 0].get_gridspec()

    for ax in axs[:, -1]:
        ax.remove()

    dr_3d_axs = fig.add_subplot(gs[:, -1], projection='3d')
    dr_3d_axs.plot(x, y, z)
    dr_3d_axs.scatter(x[0], y[0], z[0], c='g', marker='o', label='Start')
    dr_3d_axs.scatter(x[-1], y[-1], z[-1], c='r', marker='x', label='End')
    dr_3d_axs.set_xlabel('x (m)')
    dr_3d_axs.set_ylabel('y (m)')
    dr_3d_axs.set_zlabel('z (m)')
    dr_3d_axs.legend()

    axs[0, 0].plot(x, label='x')
    axs[0, 0].plot(y, label='y')
    axs[0, 0].plot(z, label='z')
    axs[0, 0].legend()
    axs[0, 0].set_xlabel('Time (a.u.)')
    axs[0, 0].set_ylabel('Position (m)')

    axs[1, 0].plot(roll, label='roll')
    axs[1, 0].plot(pitch, label='pitch')
    axs[1, 0].plot(yaw, label='yaw')
    axs[1, 0].legend()
    axs[1, 0].set_xlabel('Time (a.u.)')
    axs[1, 0].set_ylabel('Angle (rad)')

    axs[2, 0].plot(pub_roll, label='roll')
    axs[2, 0].plot(pub_pitch, label='pitch')
    axs[2, 0].plot(pub_yaw, label='yaw')
    axs[2, 0].legend()
    axs[2, 0].set_xlabel('Time (a.u.)')
    axs[2, 0].set_ylabel('Angle (rad)')

    plt.show()


if __name__ == '__main__':
    main()

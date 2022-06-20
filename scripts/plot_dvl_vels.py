#!/usr/bin/env python
import argparse
import sys

import matplotlib.pyplot as plt
import rosbag
import rospy
# from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import String
from waterlinked_a50_ros_driver.msg import DVL, DVLBeam
from scipy.spatial.transform import Rotation as R


def main():
    rospy.init_node('dvl_log_read', anonymous=False)
    bag_file = rospy.get_param('~bag', '')

    # Create 3d figure
    fig3d = plt.figure()
    ax3d = fig3d.add_subplot(111, projection='3d')

    fig2, axs2 = plt.subplots(2, 1, figsize=(10,10), sharex=True)

    fig, axs = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    vxs, vys, vzs = [], [], []
    beam1_d, beam2_d, beam3_d, beam4_d = [], [], [], []
    beam1_v, beam2_v, beam3_v, beam4_v = [], [], [], []

    xs, ys, zs = [], [], []
    roll, pitch, yaw = [], [], []

    first = True

    with rosbag.Bag(bag_file) as bag:
        for topic, msg, t in bag.read_messages(topics=['/dvl/local_position', '/dvl/data']):
            if topic == '/dvl/data':

                # Log the velocities in the body frame of the robot
                vx = -msg.velocity.y
                vy = -msg.velocity.x
                vz = msg.velocity.z
                vxs.append(vx)
                vys.append(vy)
                vzs.append(vz)

                # Log the beams
                beams = msg.beams
                beam1_v.append(beams[0].velocity)
                beam2_v.append(beams[1].velocity)
                beam3_v.append(beams[2].velocity)
                beam4_v.append(beams[3].velocity)

                beam1_d.append(beams[0].distance)
                beam2_d.append(beams[1].distance)
                beam3_d.append(beams[2].distance)
                beam4_d.append(beams[3].distance)

            if topic == '/dvl/local_position':
                try:
                    if first:
                        print('x_0: %f, y_0: %f, z_0: %f' % (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
                        first = False
                    # Means that the dead reckoning is being received
                    x = msg.pose.pose.position.x + 2507.098615
                    y = msg.pose.pose.position.y - 852.451235
                    z = msg.pose.pose.position.z + 376.106038
                    qx = msg.pose.pose.orientation.x
                    qy = msg.pose.pose.orientation.y
                    qz = msg.pose.pose.orientation.z
                    qw = msg.pose.pose.orientation.w

                    # Convert quaternion to euler angles
                    r = R.from_quat([qx, qy, qz, qw])
                    euler = r.as_euler('xyz')
                    roll.append(euler[0])
                    pitch.append(euler[1])
                    yaw.append(euler[2])

                    # Log the position in the world frame
                    xs.append(x)
                    ys.append(y)
                    zs.append(z)

                except Exception as e:
                    print(e)
                    sys.exit()

    ax3d.plot(xs, ys, zs, 'b-', label='Dead Reckoning')
    ax3d.set_xlabel('x')
    ax3d.set_ylabel('y')
    ax3d.set_zlabel('z')
    ax3d.legend()
    fig3d.suptitle('Dead reckoning')

    axs2[0].plot(xs, 'b-', label='x')
    axs2[0].plot(ys, 'r-', label='y')
    axs2[0].plot(zs, 'g-', label='z')
    axs2[0].set_ylabel('Position (m)')
    axs2[0].legend()

    axs2[1].plot(roll, 'b-', label='roll')
    axs2[1].plot(pitch, 'r-', label='pitch')
    axs2[1].plot(yaw, 'g-', label='yaw')
    axs2[1].set_ylabel('Angle (rad)')
    axs2[1].legend()
    fig2.suptitle('Dead reckoning')


    axs[0].set_title('Velocities (Body Frame)')
    # Change the title font size
    axs[0].title.set_fontsize(10)
    axs[0].set_ylabel('Velocity (m/s)')
    axs[0].plot(vxs, label=r'$v_x$')
    axs[0].plot(vys, label=r'$v_y$')
    axs[0].plot(vzs, label=r'$v_z$')
    axs[0].legend()

    axs[1].set_title('Velocities (Beams)')
    axs[1].set_xlabel('Time (a.u.)')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].title.set_fontsize(10)

    axs[1].plot(beam1_v, label='Beam 1 Velocity')
    axs[1].plot(beam2_v, label='Beam 2 Velocity')
    axs[1].plot(beam3_v, label='Beam 3 Velocity')
    axs[1].plot(beam4_v, label='Beam 4 Velocity')
    axs[1].legend()

    plt.show()


if __name__ == '__main__':
    main()

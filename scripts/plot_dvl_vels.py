#!/usr/bin/env python
import argparse
import sys

import matplotlib.pyplot as plt
import rosbag
import rospy
# from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import String
from waterlinked_a50_ros_driver.msg import DVL, DVLBeam
from ping_sonar.msg import SonarEcho
from scipy.spatial.transform import Rotation as R


def main():
    rospy.init_node('dvl_log_read', anonymous=False)
    bag_file = rospy.get_param('~bag', '')

    # Create 3d figure
    dr_fig = plt.figure()
    dr_ax = dr_fig.add_subplot(111, projection='3d')

    dr_marg_fig, dr_marg_ax = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    vels_fig, vels_axs = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    altitude_fig, altitude_ax = plt.subplots(1, 1, figsize=(10, 10))

    vxs, vys, vzs = [], [], []
    beam1_d, beam2_d, beam3_d, beam4_d = [], [], [], []
    beam1_v, beam2_v, beam3_v, beam4_v = [], [], [], []
    dvl_altitude, ping_depth = [], []
    dvl_time, ping_time = [], []

    xs, ys, zs = [], [], []
    roll, pitch, yaw = [], [], []

    first = True
    first_x, first_y, first_z = 0, 0, 0

    with rosbag.Bag(bag_file) as bag:
        for topic, msg, t in bag.read_messages(topics=['/dvl/local_position', '/dvl/data', '/ping_sonar/data']):
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

                altitude = msg.altitude
                dvl_altitude.append(altitude)
                time = msg.header.stamp.to_sec()
                dvl_time.append(time)

            if topic == '/dvl/local_position':
                try:
                    if first:
                        first_x = msg.pose.pose.position.x
                        first_y = msg.pose.pose.position.y
                        first_z = msg.pose.pose.position.z
                        rospy.loginfo('First position: {} m'.format((first_x, first_y, first_z)))
                        first = False
                    # Means that the dead reckoning is being received
                    x = msg.pose.pose.position.x - first_x
                    y = msg.pose.pose.position.y - first_y
                    z = msg.pose.pose.position.z - first_z
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
                
            if topic == '/ping_sonar/data':
                depth = msg.distance / 1000
                condfidence = msg.confidence
                ping_depth.append(depth)
                ping_timestamp = msg.header.stamp.to_sec()
                ping_time.append(ping_timestamp)

    dr_ax.plot(xs, ys, zs, 'b', label='Dead Reckoning')
    dr_ax.scatter(xs[0], ys[0], zs[0], 'g', label='Start Point')
    dr_ax.scatter(xs[-1], ys[-1], zs[-1], 'red', label='End Point')
    dr_ax.set_xlabel('x (m)')
    dr_ax.set_ylabel('y (m)')
    dr_ax.set_zlabel('z (m)')
    dr_ax.legend()
    dr_fig.suptitle('Dead reckoning')

    dr_marg_ax[0].plot(xs, 'b-', label='x')
    dr_marg_ax[0].plot(ys, 'r-', label='y')
    dr_marg_ax[0].plot(zs, 'g-', label='z')
    dr_marg_ax[0].set_title('Position')
    dr_marg_ax[0].set_ylabel('Position (m)')
    dr_marg_ax[0].legend()

    dr_marg_ax[1].plot(roll, 'b-', label='roll')
    dr_marg_ax[1].plot(pitch, 'r-', label='pitch')
    dr_marg_ax[1].plot(yaw, 'g-', label='yaw')
    dr_marg_ax[1].set_ylabel('Orientation (rad)')
    dr_marg_ax[1].set_title('Orientation')
    dr_marg_ax[1].legend()
    dr_marg_fig.suptitle('Dead reckoning')

    vels_axs[0].set_title('Velocities (Body Frame)')
    # Change the title font size
    vels_axs[0].title.set_fontsize(10)
    vels_axs[0].set_ylabel('Velocity (m/s)')
    vels_axs[0].plot(vxs, label=r'$v_x$')
    vels_axs[0].plot(vys, label=r'$v_y$')
    vels_axs[0].plot(vzs, label=r'$v_z$')
    vels_axs[0].legend()

    vels_axs[1].set_title('Velocities (Beams)')
    vels_axs[1].set_xlabel('Time (a.u.)')
    vels_axs[1].set_ylabel('Velocity (m/s)')
    vels_axs[1].title.set_fontsize(10)

    vels_axs[1].plot(beam1_v, label='Beam 1 Velocity')
    vels_axs[1].plot(beam2_v, label='Beam 2 Velocity')
    vels_axs[1].plot(beam3_v, label='Beam 3 Velocity')
    vels_axs[1].plot(beam4_v, label='Beam 4 Velocity')
    vels_axs[1].legend()

    altitude_ax.set_title('Altitude')
    altitude_ax.set_xlabel('Time (sec)')
    altitude_ax.set_ylabel('Altitude (m)')
    altitude_ax.plot(dvl_time, dvl_altitude, label='DVL')
    altitude_ax.plot(ping_time, ping_depth, label='Ping')
    altitude_ax.legend()

    plt.show()


if __name__ == '__main__':
    main()

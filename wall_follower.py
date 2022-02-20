#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from pid_control import
from sensor_msgs.msg import LaserScan
import numpy as np

max_speed = 4 #meters/second
max_steering_angle = 0.34 #rads
desired_distance = rospy.get_param('/desired_distance', 0.1) #meter; this is the distance the racecar should maintain from the wall
velocity = rospy.get_param('/velocity', 0.5) #meters/second; this is the speed of the racecar
side = rospy.get_param('/side', 1) #which side the wall being followed is at.

pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10)


def callback(data):
    if side == 1:
        lidar_data = np.array(data.ranges[90:135])
    else:
        lidar_data = np.array(data.ranges[270:315])
    rospy.loginfo(data.ranges[270])
    rospy.loginfo(data.ranges[90])
    command = AckermannDriveStamped()
    command.header.stamp = rospy.Time.now()
    # TODO do I need to use a different frame?
    command.header.frame_id = "base_link"
    command.drive.steering_angle = 0
    command.drive.steering_angle_velocity = 0
    command.drive.speed = 0.5
    command.drive.acceleration = 0.1
    command.drive.jerk = 0
    pub.publish(command)
    rate.sleep()


def follower():
    global rate
    rospy.init_node('drive_command_publisher')
    rate = rospy.Rate(20)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        follower()
    except rospy.ROSInterruptException:
        pass

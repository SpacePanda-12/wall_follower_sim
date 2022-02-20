#!/usr/bin/env python2
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        self.pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.rate = rospy.Rate(20)
        pass

    def callback(self, data):
        if self.SIDE == 1:
            lidar_data = np.array(data.ranges[90:135])
        else:
            lidar_data = np.array(data.ranges[270:315])

        # 90 degrees corresponds to left, 270 corresponds to right
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
        self.pub.publish(command)
        self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()


#!/usr/bin/env python2
import math

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *
from rospy.numpy_msg import numpy_msg



class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("racecar_simulator/scan_topic")
    WALL_TOPIC = "/wall"
    DRIVE_TOPIC = rospy.get_param("racecar_simulator/drive_topic")
    SIDE = rospy.get_param("wall_follower/side") # +1 = left, -1 = right
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    error_sum = 0

    def __init__(self):
        self.pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        # self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.rate = rospy.Rate(20)
        self.previous_time = 0
        self.previous_error = 0
        pass

    def callback(self, data):
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_inc = data.angle_increment
        if self.SIDE == -1:
            # right side (negative angles)
            # this includes data points 16 through 31; 32 is cut off, same as 83 in the other case. len(lidar_data) = 16
            lidar_data = np.array(data.ranges[22:38])
            start_angle = angle_min + 22 * angle_inc

        else:
            # left side (positive angles)
            lidar_data = np.array(data.ranges[65:85])
            start_angle = angle_min + 65 * angle_inc


        # remove outliers
        mean = np.mean(lidar_data)
        std = np.std(lidar_data)
        lidar_data = lidar_data[lidar_data < mean + std*1.1]

        # going to pick a point desired distance away from median of wall line and make robot go there
        # need to convert from polar to cartesian

        #TODO error needs to be distance from median of line (see wikipedia for equation)
        # slope should be zero when aligned with the wall
        # error term proportional to the slope of the line


        angles = np.linspace(start_angle, start_angle + angle_inc * lidar_data.size, lidar_data.size)

        # x = r cos(theta), y = r sin(theta)
        x_coords_raw = np.multiply(lidar_data, np.cos(angles))
        y_coords_raw = np.multiply(lidar_data, np.sin(angles))


        lin_reg = np.polyfit(x_coords_raw, y_coords_raw, deg=1, w=1/(np.sqrt(np.square(x_coords_raw) + np.square(y_coords_raw))))
        x_coords = []
        y_coords = []
        x_min = x_coords_raw[0]
        x_max = x_coords_raw[-1]
        x_diff = x_max - x_min
        x_interval = x_diff / lidar_data.size

        x_coords = np.linspace(x_min, x_min + x_interval * lidar_data.size, lidar_data.size)
        y_coords = lin_reg[0] * x_coords + lin_reg[1]

        VisualizationTools.plot_line([x_coords[0], x_coords[-1]], [y_coords[0], y_coords[-1]], self.line_pub, frame="/laser")

        # polyfit returns numpy array for y = mx+b; [m, b] is what is returned.
        # assuming kc = 0.5
        # p_gain = 15
        p_gain = 20
        d_gain = 0.5
        i_gain = 0

        current_time = rospy.get_time()

        error =  abs(lin_reg[1]/math.sqrt(np.square(lin_reg[0])+1)) - self.DESIRED_DISTANCE


        if self.previous_time == 0:
            dt = 0
            robot_command = p_gain * error
        else:
            dt = current_time - self.previous_time
            self.error_sum += dt * error

            if self.SIDE == 1:
                robot_command = (p_gain * (error + 2.2 * lin_reg[0]) + d_gain * (
                            error - self.previous_error) / dt + i_gain * self.error_sum)
            if self.SIDE == -1:
                robot_command = -(p_gain * (error - 2.2 * lin_reg[0]) + d_gain * (
                            error - self.previous_error) / dt + i_gain * self.error_sum)

        #
        # if lin_reg[0] > 1 & self.SIDE == -1:
        #     robot_command = 10

        self.previous_error = error

        rospy.loginfo(lin_reg[0])

        self.previous_time = current_time

        command = AckermannDriveStamped()
        command.header.stamp = rospy.Time.now()
        # TODO do I need to use a different frame?
        command.header.frame_id = "base_link"
        command.drive.steering_angle = robot_command
        command.drive.steering_angle_velocity = 0
        command.drive.speed = self.VELOCITY
        command.drive.acceleration = 0
        command.drive.jerk = 0
        self.pub.publish(command)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()


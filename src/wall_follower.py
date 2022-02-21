#!/usr/bin/env python2
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
    SIDE = rospy.get_param("racecar_simulator/side", -1) # +1 = left, -1 = right
    VELOCITY = rospy.get_param("racecar_simulator/velocity", 0.5)
    DESIRED_DISTANCE = rospy.get_param("racecar_simulator/desired_distance", 1)

    def __init__(self):
        self.pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        # self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.rate = rospy.Rate(20)
        self.previous_time = 0
        pass

    def callback(self, data):
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_inc = data.angle_increment
        if self.SIDE == -1:
            # right side (negative angles)
            # this includes data points 16 through 31; 32 is cut off, same as 83 in the other case. len(lidar_data) = 16
            lidar_data = np.array(data.ranges[16:32])
            start_angle = angle_min + 16 * angle_inc

        else:
            # left side (positive angles)
            lidar_data = np.array(data.ranges[67:83])
            start_angle = angle_min + 67 * angle_min

        # going to pick a point desired distance away from median of wall line and make robot go there
        # need to convert from polar to cartesian

        angles = []
        for i in range(16):
            angles.append(start_angle + angle_inc * i)
        angles = np.array(angles)
        # x = r cos(theta), y = r sin(theta)
        x_coords = np.multiply(lidar_data * np.cos(angles))
        y_coords = np.multiply(lidar_data * np.sin(angles))
        VisualizationTools.plot_line(x_coords, y_coords, self.line_pub, frame="/laser")




        # polyfit returns numpy array for y = mx+b; [m, b] is what is returned.
        lin_reg = np.polyfit(angles, lidar_data, deg=1)

        offset_error = 0
        angle_error = 0
        p_gain = 0.05
        d_gain = 0.06
        position_error = 0
        angular_error = 0


        current_time = rospy.get_time()
        if self.previous_time == 0:
            dt = 0
            steering_angle_command = p_gain * error
        else:
            dt = current_time - self.previous_time
            steering_angle_command = p_gain * error + d_gain * error / dt
        self.previous_time = current_time






        command = AckermannDriveStamped()
        command.header.stamp = rospy.Time.now()
        # TODO do I need to use a different frame?
        command.header.frame_id = "base_link"
        command.drive.steering_angle = 0.1
        command.drive.steering_angle_velocity = 0
        command.drive.speed = self.VELOCITY
        command.drive.acceleration = 0
        command.drive.jerk = 0
        self.pub.publish(command)
        self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()


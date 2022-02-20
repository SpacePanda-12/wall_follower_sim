#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped


def simple_publisher():
    pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10)
    rospy.init_node('drive_command_publisher')
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
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

if __name__ == '__main__':
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        pass

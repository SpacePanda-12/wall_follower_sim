cmake_minimum_required(VERSION 2.8.3)
project(wall_follower)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)


catkin_package()

catkin_install_python(PROGRAMS
   src/wall_follower.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


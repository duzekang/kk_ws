cmake_minimum_required(VERSION 3.0.2)
project(trajectory_visualizer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/trajectory_visualizer_node.py
  scripts/send_traj_signal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

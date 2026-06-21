cmake_minimum_required(VERSION 3.0.2)
project(yolov8_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  yolov8_ros_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge yolov8_ros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/yolo_v8.py
  scripts/yolo_v8_new.py
  scripts/ros_predict.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install weights
install(DIRECTORY weights/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/weights
)

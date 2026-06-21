cmake_minimum_required(VERSION 3.0.2)
project(radar_curve_planner)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(radar_midpoint_node src/radar_midpoint_node.cpp)
add_dependencies(radar_midpoint_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_midpoint_node ${catkin_LIBRARIES})

add_executable(curve_yaml_writer_node src/curve_yaml_writer_node.cpp)
add_dependencies(curve_yaml_writer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(curve_yaml_writer_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/view_yaml_traj.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS radar_midpoint_node curve_yaml_writer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

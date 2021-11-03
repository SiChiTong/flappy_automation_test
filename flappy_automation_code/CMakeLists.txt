cmake_minimum_required(VERSION 2.8.3)
project(flappy_automation_code)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(${PROJECT_NAME}_node src/flappy_automation_code.cpp)
#add_executable(${flappy_laser_to_points}_node src/flappy_laser_to_points.cpp)
#
#target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#)

#target_link_libraries(${flappy_laser_to_points}_node
#   ${catkin_LIBRARIES}
#)

add_executable(flappy_automation_code_node src/flappy_automation_code.cpp)
target_link_libraries(flappy_automation_code_node ${catkin_LIBRARIES})
add_dependencies(flappy_automation_code_node flappy_automation_code_cpp)

add_executable(flappy_laser_to_points_node src/flappy_laser_to_points.cpp)
target_link_libraries(flappy_laser_to_points_node ${catkin_LIBRARIES})
add_dependencies(flappy_laser_to_points_node flappy_laser_to_points_cpp)

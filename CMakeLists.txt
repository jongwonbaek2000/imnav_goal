cmake_minimum_required(VERSION 3.0.2)
project(imnav_goal)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

find_package(GeographicLib REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imnav_goal
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
  DEPENDS GeographicLib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_executable(imnav_goal_node src/imnav_goal_node.cpp)
target_link_libraries(imnav_goal_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

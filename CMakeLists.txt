cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(pepper_imitation VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wl,--no-undefined")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  geometry_msgs
  tf
  moveit_ros_planning_interface
)

catkin_package(CATKIN_DEPENDS)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_node src/pepper_imitation_node.cpp src/PepperImitationNode.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

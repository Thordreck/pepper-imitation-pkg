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
  std_msgs
  sensor_msgs
  tf
  moveit_ros_planning_interface
  message_generation
  naoqi_libqi
  naoqi_libqicore
)

add_message_files(
    FILES
    ImitationPose.msg
    ImitationResult.msg
    AudioPlayerCommand.msg
    UserCommand.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${NAOqi_INCLUDE_DIRS}
  include
)

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})

add_executable(${PROJECT_NAME}_node src/pepper_imitation_node.cpp src/PepperImitationNode.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_tts_node src/pepper_tts_node.cpp src/PepperTTSNode.cpp)
target_link_libraries(${PROJECT_NAME}_tts_node ${catkin_LIBRARIES} ${naoqi_libqi_LIBRARIES})

add_executable(${PROJECT_NAME}_audio_player_node src/pepper_audio_player_node.cpp src/PepperAudioPlayerNode.cpp)
target_link_libraries(${PROJECT_NAME}_audio_player_node ${catkin_LIBRARIES} ${naoqi_libqi_LIBRARIES})
add_dependencies(${PROJECT_NAME}_audio_player_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_tablet_node src/pepper_tablet_node.cpp src/PepperTabletNode.cpp)
target_link_libraries(${PROJECT_NAME}_tablet_node ${catkin_LIBRARIES} ${naoqi_libqi_LIBRARIES})
add_dependencies(${PROJECT_NAME}_tablet_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_face_tracker_node src/pepper_face_tracker_node.cpp src/PepperFaceTrackerNode.cpp)
target_link_libraries(${PROJECT_NAME}_face_tracker_node ${catkin_LIBRARIES} ${naoqi_libqi_LIBRARIES})
add_dependencies(${PROJECT_NAME}_face_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_teleop_joy_node src/pepper_teleop_joy_node.cpp src/PepperTeleopJoyNode.cpp)
target_link_libraries(${PROJECT_NAME}_teleop_joy_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_teleop_joy_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

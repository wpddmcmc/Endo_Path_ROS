cmake_minimum_required(VERSION 2.8.3)
project(detector)
 
# using C++11 
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")

# if u have OpenCV version more than one, set the build path which one u want to use
set(OpenCV_DIR "/home/tgeek/APPSource/opencv-3.4.6/build")
 

find_package(catkin REQUIRED COMPONENTS
OpenCV REQUIRED
roscpp
roslib
cv_bridge
image_transport
std_msgs
message_generation
)
 
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

add_message_files(
  FILES
  Detector_Info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport message_runtime
)
 
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/capture_node/include
  ${CMAKE_CURRENT_SOURCE_DIR}/stream_node/include
  /usr/local/cuda-9.0/include   
  include
  ${catkin_INCLUDE_DIRS}
)
 
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/capture_node/src/ capture_SRCS)
add_executable(capture_node ${capture_SRCS})
target_link_libraries(capture_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/stream_node/src/ stream_SRCS)
add_executable(stream_node ${stream_SRCS})
target_link_libraries(stream_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/listener_node/src/ listen_SRCS)
add_executable(listener_node ${listen_SRCS})
target_link_libraries(listener_node ${catkin_LIBRARIES})

add_dependencies(stream_node detector_generate_messages_cpp)
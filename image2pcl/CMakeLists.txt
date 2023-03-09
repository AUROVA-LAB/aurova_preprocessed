cmake_minimum_required(VERSION 3.0.2)
project(image2pcl)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS "-std=gnu++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  detection_msgs
  eigen_conversions
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(ouster_filtered_yolo_node  src/ouster_filtered_yolo.cpp)
add_executable(image2pcl_node  src/image2pcl.cpp)
add_executable(tracker_filtered_node  src/tracker_filtered.cpp)
target_link_libraries(image2pcl_node  
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)
target_link_libraries(ouster_filtered_yolo_node 
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(tracker_filtered_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)








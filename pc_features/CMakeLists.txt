cmake_minimum_required(VERSION 3.5)
project(pc_features)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####For docker installation only, todo remove this dependencies to local installation
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "/usr/local/lib/localization;/share_libs/")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(detection_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(pc_feature_node  src/pc2image.cpp)
add_executable(zdepthdata_node  src/pc2image_z_depth_data.cpp)

target_link_libraries(pc_feature_node  
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
target_link_libraries(zdepthdata_node  
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

ament_target_dependencies(pc_feature_node detection_msgs rclcpp tf2 tf2_ros PCL cv_bridge message_filters pcl_conversions image_transport)
ament_target_dependencies(zdepthdata_node detection_msgs rclcpp tf2 tf2_ros PCL cv_bridge message_filters pcl_conversions image_transport)


# target_link_libraries(ouster_filtered_yolo_node 
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    ${Boost_LIBRARIES}
# )

# target_link_libraries(tracker_filtered_node
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    ${Boost_LIBRARIES}
# )

install(TARGETS
  pc_feature_node
  zdepthdata_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  pc_features/filter_PCL.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp rclpy Eigen detection_msgs)
ament_package()
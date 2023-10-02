### Example of usage:

You can run an example following the instructions in [applications](https://github.com/AUROVA-LAB/applications) (Examples).

# aurova_preprocessed
This is a metapackage that contains different packages that perform processes related to the preprocessing of data read from different types of sensors. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "name_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**virtual_imu**
This package contains a node that, as input, read the topic /imu/data of type sensor_msgs::Imu. This node generate a new sensor_msgs::Imu that contains the estimation of orientation integrating rpy. The node output is published in the topic /virtual_imu_data.

**lidar_camera_fusion**
The code implemented in ROS projects a point cloud obtained by a Velodyne VLP16 3D-Lidar sensor on an image from an RGB camera. The example used the ROS package to calibrate a camera and a LiDAR from lidar_camera_calibration.

**pc2image**
This code converts a point cloud obtained by a Velodyne VLP16 3D-Lidar sensor into a depth image mono16. This package is used for the preprocessing of Lidar odometry (LiLo).

**pc_filter**
This node filters the range image from the pc2image node and converts the range image into a point cloud. 

**merged_channels_ouster**
This node merges the nearir, signal and reflec channels published by the ouster sensor node.

**lidar_obstacle_detection**.
Obstacle detection node with ouster lidar. It works by calculating the normals in two ranges (small and large) and then filtering them **(node in test version)**

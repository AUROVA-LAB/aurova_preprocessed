# PointCloud on Image
The code implemented in ROS projects a point cloud obtained by a Velodyne VLP16 3D-Lidar sensor on an image from an RGB camera. The example used the ROS package to calibrate a camera and a LiDAR from [lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration).

## Requisites
- [ROS](http://wiki.ros.org/ROS/Installation) Kinetic or Melodic
- [Velodyne](https://github.com/ros-drivers/velodyne) repository
- [PCL](https://pointclouds.org/) (Point Cloud Library)
- [Armadillo](http://arma.sourceforge.net/download.html)

## Ros Launch
```
  roslaunch lidar_camera_fusion vlp16OnImg.launch 
```
## Test 


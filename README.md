### Example of usage:

You can run an example following the instructions in [applications](https://github.com/AUROVA-LAB/applications) (Examples).

# aurova_preprocessed
This is a metapackage that contains different packages that perform processes related to the preprocessing of data read from different types of sensors. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "name_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**pc2image** [deprecated] (use [pc_feature](https://github.com/AUROVA-LAB/aurova_preprocessed/tree/master/pc_features) node instead.)

This code converts a point cloud obtained by a Velodyne VLP16 or an Ouster OS1-128 3D-Lidar sensor into a mono16 range image. In addition, filters are applied on the image to extract edges, surfaces and the ground plane in the depth image. These filtered images can then be reconstructed back into point clouds. This package works in conjunction with the [odom_estimation_pc](https://github.com/AUROVA-LAB/aurova_odom/tree/main/odom_estimation_pc) package as a preprocessor for LiDAR odometry (LiLo).

## ✨ License
This work is licensed under a [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-nc-sa/4.0) and is intended for non-commercial academic use. If you are interested in using the software for commercial purposes, don't hesitate to get in touch with us via [email](mailto:miguelangel.munoz@ua.es).

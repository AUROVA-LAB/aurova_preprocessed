# aurova_preprocessed
This is a metapackage that contains different packages that perform processes related to the preprocessing of data read from different types of sensors. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "name_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**ackermann_to_odom**
This package contains a node that, as input, reads the topics /estimated_ackermann_state and /covariance_ackermann_state, of type ackermann_msgs::AckermannDriveStamped, and /virtual_imu_data of type sensor_msgs::Imu. This node parse this information as a new message type nav_msgs::Odometry using the 2D tricicle model. This message is published in an output topic called /odometry.
* ~odom_in_tf (default: false): If this parameter is set to true, the odometry is also published in /tf topic.
* ~scan_in_tf (default: false): If this parameter is set to true, the laser transform read from the static robot transformation is published in /tf topic.
* ~frame_id (default: ""): This parameter is the name of frame to transform if scan_in_tf is true.
* ~child_id (default: ""): This parameter is the name of child frame to transform if scan_in_tf is true.

**gps_to_odom**
This package contains a node that, as input, reads the topics /odometry_gps_fix, of type nav_msgs::Odometry, and /rover/fix_velocity of type geometry_msgs::TwistWithCovariance. This node calculate the orientation using the velocities from gps, and generate new odometry (message type  type nav_msgs::Odometry) with the information provided by /odometry_gps_fix. This message is published in output topic called /odometry_gps.

**virtual_imu**
This package contains a node that, as input, read the topic /imu/data of type sensor_msgs::Imu. This node generate a new sensor_msgs::Imu that contains the estimation of orientation integrating rpy. The node output is published in the topic /virtual_imu_data. 

# aurova_preprocessed
This is a metapackage that contains different packages that perform processes related to the preprocessing of data read from different types of sensors. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "name_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**ackermann_to_odom**
This package contains a node that, as input, reads the topics /estimated_ackermann_state and /covariance_ackermann_state, of type ackermann_msgs::AckermannDriveStamped, and /virtual_imu_data of type sensor_msgs::Imu. This node parse this information as a new message type nav_msgs::Odometry, which is published in an output topic called /odometry.

**virtual_imu**
This package contains a node that, as input, read the topic /rover/fix_velocity, of type geometry_msgs::TwistWithCovarianceStamped, and /imu/data of type sensor_msgs::Imu. This node generate a new sensor_msgs::Imu that contains the estimation of orientation integrating rpy a fix_velocity with a kalman filter. The node output is published in the topic /virtual_imu_data. 

### Example of usage:

You can run an example following the instructions in [applications](https://github.com/AUROVA-LAB/applications) (Examples).

# aurova_preprocessed
This is a metapackage that contains different packages that perform processes related to the preprocessing of data read from different types of sensors. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "name_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**virtual_imu**
This package contains a node that, as input, read the topic /imu/data of type sensor_msgs::Imu. This node generate a new sensor_msgs::Imu that contains the estimation of orientation integrating rpy. The node output is published in the topic /virtual_imu_data.

**dump_imu_data_for_calibration_with_imutk**
This package contains a node that takes as input the topic /imu/data and generates two files (one for linear accelerations  and other for angular rates) in the format required for the software imu_tk (https://github.com/AUROVA/imu_tk).

Parameters:
* ~/dump_imu_data_for_calibration_with_imutk/accelerometer_output_file_path (default: ""): Path for the acc file.
* ~/dump_imu_data_for_calibration_with_imutk/gyroscope_output_file_path (default: ""):     Path for the gyro file.


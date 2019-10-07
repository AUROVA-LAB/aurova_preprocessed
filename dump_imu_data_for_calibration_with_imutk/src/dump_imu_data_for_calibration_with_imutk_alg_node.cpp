#include "dump_imu_data_for_calibration_with_imutk_alg_node.h"

DumpImuDataForCalibrationWithImutkAlgNode::DumpImuDataForCalibrationWithImutkAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<DumpImuDataForCalibrationWithImutkAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 20;//in [Hz]

  // [init publishers]
  
  // [init subscribers]
  this->imu_ = this->public_node_handle_.subscribe("/imu/data", 1, &DumpImuDataForCalibrationWithImutkAlgNode::cb_imuData, this);

  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  std::cout << "Creating output files " << std::endl;

  std::string acc_filename;
  this->public_node_handle_.getParam("/dump_imu_data_for_calibration_with_imutk/accelerometer_output_file_path", acc_filename);

  std::string gyro_filename;
  this->public_node_handle_.getParam("/dump_imu_data_for_calibration_with_imutk/gyroscope_output_file_path", gyro_filename);

  assert(!acc_filename.empty() && !gyro_filename.empty() && "Error, path for output files not specified!, please set those params." );

  acc_results_file_.open(acc_filename.c_str(), std::ofstream::trunc);
  gyro_results_file_.open(gyro_filename.c_str(), std::ofstream::trunc);

  std::cout << "Output files created!" << std::endl;

  flag_first_time_stamp_received_ = false;
  first_timestamp_ = 0.0;

  static_interval_id_  = -1;
  flag_recording_data_ = false;
  number_of_static_samples_in_current_interval_    = 0;
  number_of_transient_samples_in_current_interval_ = 0;
}

DumpImuDataForCalibrationWithImutkAlgNode::~DumpImuDataForCalibrationWithImutkAlgNode(void)
{
  // [free dynamic memory]

  assert(!acc_data_ready_to_be_written_to_file_.empty() && !gyro_data_ready_to_be_written_to_file_.empty()
         && "Error, no imu data received!, nothing to save, check that the IMU is detected as /dev/imu" );

  acc_results_file_  << acc_data_ready_to_be_written_to_file_;
  gyro_results_file_ << gyro_data_ready_to_be_written_to_file_;

  std::cout << acc_data_ready_to_be_written_to_file_;

  std::cout << "Closing output files " << std::endl;

  acc_results_file_.close();
  gyro_results_file_.close();

}

void DumpImuDataForCalibrationWithImutkAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]

  if(!flag_first_time_stamp_received_)
  {
    ROS_WARN_STREAM("Waiting for imu data...");
  }

  if (flag_recording_data_ && number_of_static_samples_in_current_interval_ % 10 <= 2)
  {
    std::cout << "Static IMU samples gathered in current interval: " << number_of_static_samples_in_current_interval_ << std::endl;
  }


  if (!flag_recording_data_ && number_of_transient_samples_in_current_interval_ % 10 <= 2)
  {
    std::cout << "Transient IMU samples between static intervals: " << number_of_transient_samples_in_current_interval_ << std::endl;
  }
}

/*  [subscriber callbacks] */
void DumpImuDataForCalibrationWithImutkAlgNode::cb_imuData(const sensor_msgs::Imu& Imu_msg)
{
  this->alg_.lock();

  if(!flag_first_time_stamp_received_)
  {
    flag_first_time_stamp_received_ = true;
    first_timestamp_ = Imu_msg.header.stamp.sec + (Imu_msg.header.stamp.nsec * 1e-9);
    std::cout << "First imu data received!!" << std::endl;
  }


  double current_timestamp = Imu_msg.header.stamp.sec + (Imu_msg.header.stamp.nsec * 1e-9);

  std::setiosflags(std::ios::fixed);
  std::setprecision(11);
  std::ostringstream s_gyro;
  std::ostringstream s_acc;

  int id = -1; //code for transitions
  if(flag_recording_data_) id = static_interval_id_; //code for IMU data generated in stationary position

  s_gyro << current_timestamp - first_timestamp_
         << "," << Imu_msg.angular_velocity.x
         << "," << Imu_msg.angular_velocity.y
         << "," << Imu_msg.angular_velocity.z
         << "," << id
         << std::endl;

  gyro_data_ready_to_be_written_to_file_ += s_gyro.str();

  s_acc  << current_timestamp - first_timestamp_
         << "," << Imu_msg.linear_acceleration.x
         << "," << Imu_msg.linear_acceleration.y
         << "," << Imu_msg.linear_acceleration.z
         << "," << id
         << std::endl;

  acc_data_ready_to_be_written_to_file_ += s_acc.str();

  if(flag_recording_data_)
  {
    number_of_static_samples_in_current_interval_++;
  }
  else
  {
    number_of_transient_samples_in_current_interval_++;
  }

  //std::cout << s_acc.str();

  this->alg_.unlock();
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void DumpImuDataForCalibrationWithImutkAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;

  if( flag_recording_data_ != config.recording_data)
  {
    flag_recording_data_ = config.recording_data;
    if(flag_recording_data_)
    {
      static_interval_id_++;
      std::cout << "Recording static interval number " << static_interval_id_ << std::endl;
      number_of_transient_samples_in_current_interval_ = 0;
    }
    else
    {
      std::cout << "Stop recording, static interval number " << static_interval_id_
          << " finished with " << number_of_static_samples_in_current_interval_ << " IMU samples" << std::endl;
      number_of_static_samples_in_current_interval_ = 0;
    }
  }

  this->alg_.unlock();
}

void DumpImuDataForCalibrationWithImutkAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<DumpImuDataForCalibrationWithImutkAlgNode>(argc, argv, "dump_imu_data_for_calibration_with_imutk_alg_node");
}

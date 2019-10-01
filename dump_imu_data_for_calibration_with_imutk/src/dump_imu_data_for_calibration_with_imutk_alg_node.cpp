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
  std::string acc_filename = "/home/idelpino/Documents/imu_acc.mat";
  acc_results_file_.open(acc_filename.c_str(), std::ofstream::trunc);

  std::string gyro_filename = "/home/idelpino/Documents/imu_gyro.mat";
  gyro_results_file_.open(gyro_filename.c_str(), std::ofstream::trunc);

  std::cout << "Output files created!" << std::endl;

  flag_first_time_stamp_received_ = false;
  first_timestamp_ = 0.0;
}

DumpImuDataForCalibrationWithImutkAlgNode::~DumpImuDataForCalibrationWithImutkAlgNode(void)
{
  // [free dynamic memory]

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
}

/*  [subscriber callbacks] */
void DumpImuDataForCalibrationWithImutkAlgNode::cb_imuData(const sensor_msgs::Imu& Imu_msg)
{
  this->alg_.lock();

  if(!flag_first_time_stamp_received_)
  {
    flag_first_time_stamp_received_ = true;
    first_timestamp_ = Imu_msg.header.stamp.sec + (Imu_msg.header.stamp.nsec * 1e-9);
  }

  double current_timestamp = Imu_msg.header.stamp.sec + (Imu_msg.header.stamp.nsec * 1e-9);

  std::setiosflags(std::ios::fixed);
  std::setprecision(11);
  std::ostringstream s_gyro;
  std::ostringstream s_acc;

  s_gyro << "   " << current_timestamp - first_timestamp_
         << "   " << Imu_msg.angular_velocity.x
         << "   " << Imu_msg.angular_velocity.y
         << "   " << Imu_msg.angular_velocity.z
         << std::endl;

  gyro_data_ready_to_be_written_to_file_ += s_gyro.str();

  s_acc  << "   " << current_timestamp - first_timestamp_
         << "   " << Imu_msg.linear_acceleration.x
         << "   " << Imu_msg.linear_acceleration.y
         << "   " << Imu_msg.linear_acceleration.z
         << std::endl;

  acc_data_ready_to_be_written_to_file_ += s_acc.str();

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

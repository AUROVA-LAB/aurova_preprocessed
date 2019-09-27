#include "virtual_imu_alg_node.h"

VirtualImuAlgNode::VirtualImuAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<VirtualImuAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 100; //in [Hz]
  this->originl_imu_msg_.angular_velocity.x = 0.0;
  this->originl_imu_msg_.angular_velocity.y = 0.0;
  this->originl_imu_msg_.angular_velocity.z = 0.0;
  this->virtual_imu_msg_.angular_velocity.x = 0.0;
  this->virtual_imu_msg_.angular_velocity.y = 0.0;
  this->virtual_imu_msg_.angular_velocity.z = 0.0;

  // [init publishers]
  this->imu_publisher_ = this->public_node_handle_.advertise < sensor_msgs::Imu > ("/virtual_imu_data", 1);

  // [init subscribers]
  this->original_imu_ = this->public_node_handle_.subscribe("/imu/data", 1, &VirtualImuAlgNode::cb_imuData, this);

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

VirtualImuAlgNode::~VirtualImuAlgNode(void)
{
  // [free dynamic memory]
  acc_results_file_  << acc_data_ready_to_be_written_to_file_;
  gyro_results_file_ << gyro_data_ready_to_be_written_to_file_;

  std::cout << acc_data_ready_to_be_written_to_file_;

  std::cout << "Closing output files " << std::endl;
  acc_results_file_.close();
  gyro_results_file_.close();
}

void VirtualImuAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  this->alg_.createVirtualImu(this->originl_imu_msg_, this->virtual_imu_msg_);

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->imu_publisher_.publish(this->virtual_imu_msg_);
}

/*  [subscriber callbacks] */
void VirtualImuAlgNode::cb_imuData(const sensor_msgs::Imu& Imu_msg)
{
  this->alg_.lock();
  this->originl_imu_msg_.angular_velocity.x = Imu_msg.angular_velocity.x;
  this->originl_imu_msg_.angular_velocity.y = Imu_msg.angular_velocity.y;
  this->originl_imu_msg_.angular_velocity.z = Imu_msg.angular_velocity.z;

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

void VirtualImuAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void VirtualImuAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < VirtualImuAlgNode > (argc, argv, "virtual_imu_alg_node");
}

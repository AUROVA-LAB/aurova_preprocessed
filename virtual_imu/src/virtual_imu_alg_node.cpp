#include "virtual_imu_alg_node.h"

VirtualImuAlgNode::VirtualImuAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<VirtualImuAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 20; //in [Hz]

  // [init publishers]
  this->imu_publisher_ = this->public_node_handle_.advertise < sensor_msgs::Imu > ("/virtual_imu_data", 1);

  // [init subscribers]
  this->original_imu_ = this->public_node_handle_.subscribe("/imu/data", 1, &VirtualImuAlgNode::cb_imuData, this);
  this->gps_velocity_ = this->public_node_handle_.subscribe("/rover/fix_velocity", 1,
                                                            &VirtualImuAlgNode::cb_gpsVelocity, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

VirtualImuAlgNode::~VirtualImuAlgNode(void)
{
  // [free dynamic memory]
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
  this->alg_.unlock();
}
void VirtualImuAlgNode::cb_gpsVelocity(const geometry_msgs::TwistWithCovarianceStamped& gps_msg)
{
  /*
   this->alg_.lock();
   float roll, pitch, yaw;
   this->alg_.rpyFromGpsVelocity(roll, pitch, yaw, gps_msg.twist.twist.linear.x, gps_msg.twist.twist.linear.y,
   gps_msg.twist.twist.linear.z);

   if (this->alg_.first_run_ && this->alg_.first_vel_)
   {
   ROS_INFO("first yaw: [%f]", yaw);
   this->alg_.yaw_offset_ = yaw - this->alg_.estimation_rpy_->X_[2][0];
   this->alg_.first_run_ = 0;
   }

   yaw = yaw - this->alg_.yaw_offset_;
   //this->alg_.estimation_rpy_->correct(0.0, 0.0, yaw);
   //debug
   //ROS_INFO("Info Gps: %f", gps_msg.twist.twist.linear.x);
   this->alg_.unlock();
   */
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

#include "ackermann_to_odom_alg_node.h"

AckermannToOdomAlgNode::AckermannToOdomAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<AckermannToOdomAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]

  // [init publishers]
  this->odometry_publisher_ = this->public_node_handle_.advertise < nav_msgs::Odometry > ("/odometry", 1);

  // [init subscribers]
  this->estimated_ackermann_subscriber_ = this->public_node_handle_.subscribe(
      "/estimated_ackermann_state", 1, &AckermannToOdomAlgNode::cb_ackermannState, this);
  this->covariance_ackermann_subscriber_ = this->public_node_handle_.subscribe(
      "/covariance_ackermann_state", 1, &AckermannToOdomAlgNode::cd_ackermannCovariance, this);
  this->virtual_imu_subscriber_ = this->public_node_handle_.subscribe("/virtual_imu_data", 1,
                                                                      &AckermannToOdomAlgNode::cb_imuData, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

AckermannToOdomAlgNode::~AckermannToOdomAlgNode(void)
{
  // [free dynamic memory]
}

void AckermannToOdomAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  this->alg_.generateNewOdometryMsg(this->estimated_ackermann_state_, this->covariance_,
                                    this->virtual_imu_msg_, this->odometry_, this->odom_trans_,
                                    this->base_trans_);

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->odometry_publisher_.publish(this->odometry_);
  this->broadcaster_.sendTransform(this->odom_trans_);
  this->broadcaster_.sendTransform(this->base_trans_);
}

/*  [subscriber callbacks] */
void AckermannToOdomAlgNode::cb_ackermannState(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr& estimated_ackermann_state_msg)
{
  this->alg_.lock();

  this->estimated_ackermann_state_.drive.speed = estimated_ackermann_state_msg->drive.speed;
  this->estimated_ackermann_state_.drive.steering_angle = estimated_ackermann_state_msg->drive.steering_angle;

  //debug
  //ROS_INFO("debug [%f]", estimated_ackermann_state_msg->drive.speed);
  this->alg_.unlock();
}
void AckermannToOdomAlgNode::cd_ackermannCovariance(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr& covariance_ackermann_state_msg)
{
  this->alg_.lock();

  this->covariance_ = covariance_ackermann_state_msg->drive.speed;

  //debug
  //ROS_INFO("debug [%f]", covariance_ackermann_state_msg->drive.speed);
  this->alg_.unlock();
}
void AckermannToOdomAlgNode::cb_imuData(const sensor_msgs::Imu::ConstPtr& Imu_msg)
{
  this->alg_.lock();

  this->virtual_imu_msg_.orientation.x = Imu_msg->orientation.x;
  this->virtual_imu_msg_.orientation.y = Imu_msg->orientation.y;
  this->virtual_imu_msg_.orientation.z = Imu_msg->orientation.z;
  this->virtual_imu_msg_.orientation.w = Imu_msg->orientation.w;
  //debug
  //ROS_INFO("debug [%f]", Imu_msg->orientation.x);
  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void AckermannToOdomAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void AckermannToOdomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < AckermannToOdomAlgNode > (argc, argv, "ackermann_to_odom_alg_node");
}

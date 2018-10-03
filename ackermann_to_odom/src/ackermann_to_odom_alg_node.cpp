#include "ackermann_to_odom_alg_node.h"

AckermannToOdomAlgNode::AckermannToOdomAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<AckermannToOdomAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]
  this->virtual_imu_msg_.orientation.x = 0.0;
  this->virtual_imu_msg_.orientation.y = 0.0;
  this->virtual_imu_msg_.orientation.z = 0.0;
  this->virtual_imu_msg_.orientation.w = 1.0;
  this->estimated_ackermann_state_.drive.speed = 0.0;
  this->estimated_ackermann_state_.drive.steering_angle = 0.0;

  // [init publishers]
  this->odometry_publisher_ = this->public_node_handle_.advertise < nav_msgs::Odometry > ("/odometry", 1);

  // [init subscribers]
  this->estimated_ackermann_subscriber_ = this->public_node_handle_.subscribe(
      "/estimated_ackermann_state", 1, &AckermannToOdomAlgNode::cb_ackermannState, this);
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
  bool odom_in_tf;
  float x_tf;
  float yaw_tf;
  std::string frame_id;
  std::string child_id;

  // [read parameters]
  this->public_node_handle_.getParam("/odom_in_tf", odom_in_tf);
  this->public_node_handle_.getParam("/x_tf", x_tf);
  this->public_node_handle_.getParam("/yaw_tf", yaw_tf);
  this->public_node_handle_.getParam("/frame_id", frame_id);
  this->public_node_handle_.getParam("/child_id", child_id);

  // [fill msg structures]
  this->alg_.generateNewOdometryMsg(this->estimated_ackermann_state_, this->virtual_imu_msg_, this->odometry_,
                                    this->odom_trans_);
  this->base_trans_.header.frame_id = frame_id;
  this->base_trans_.child_frame_id = child_id;
  this->base_trans_.header.stamp = ros::Time::now();
  this->base_trans_.transform.translation.x = x_tf;
  this->base_trans_.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_tf);

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  if (odom_in_tf)
  {
    this->broadcaster_.sendTransform(this->odom_trans_);
    this->broadcaster_.sendTransform(this->base_trans_);
  }
  else
  {
    this->odometry_publisher_.publish(this->odometry_);
  }
}

/*  [subscriber callbacks] */
void AckermannToOdomAlgNode::cb_ackermannState(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr& estimated_ackermann_state_msg)
{
  this->alg_.lock();

  this->estimated_ackermann_state_.drive.speed = estimated_ackermann_state_msg->drive.speed;
  this->estimated_ackermann_state_.drive.steering_angle = estimated_ackermann_state_msg->drive.steering_angle;

  this->alg_.unlock();
}

void AckermannToOdomAlgNode::cb_imuData(const sensor_msgs::Imu::ConstPtr& Imu_msg)
{
  this->alg_.lock();

  this->virtual_imu_msg_.orientation.x = Imu_msg->orientation.x;
  this->virtual_imu_msg_.orientation.y = Imu_msg->orientation.y;
  this->virtual_imu_msg_.orientation.z = Imu_msg->orientation.z;
  this->virtual_imu_msg_.orientation.w = Imu_msg->orientation.w;

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

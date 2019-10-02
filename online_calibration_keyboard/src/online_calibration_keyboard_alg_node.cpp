#include "online_calibration_keyboard_alg_node.h"

OnlineCalibrationKeyboardAlgNode::OnlineCalibrationKeyboardAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<OnlineCalibrationKeyboardAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]

  // [init publishers]
  this->delta_tf_publisher_ = this->public_node_handle_.advertise < geometry_msgs::Pose > ("/delta_tf", 1);

  // [init subscribers]
  // [init services]
  // [init clients]
  // [init action servers]
  // [init action clients]
}

OnlineCalibrationKeyboardAlgNode::~OnlineCalibrationKeyboardAlgNode(void)
{
  // [free dynamic memory]
}

void OnlineCalibrationKeyboardAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  int key = this->alg_.getch();
  this->alg_.mapKeysToVelocities(key, this->alg_.st_twist_change_calib_);
  this->delta_tf_.position.x = this->alg_.st_twist_change_calib_.x;
  this->delta_tf_.position.y = this->alg_.st_twist_change_calib_.y;
  this->delta_tf_.position.z = this->alg_.st_twist_change_calib_.z;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(this->alg_.st_twist_change_calib_.r,
                                                          this->alg_.st_twist_change_calib_.p,
                                                          this->alg_.st_twist_change_calib_.w);
  this->delta_tf_.orientation.x = quaternion[0];
  this->delta_tf_.orientation.y = quaternion[1];
  this->delta_tf_.orientation.z = quaternion[2];
  this->delta_tf_.orientation.w = quaternion[3];
  if (this->alg_.st_twist_change_calib_.flag_data)
    this->delta_tf_publisher_.publish(this->delta_tf_);
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void OnlineCalibrationKeyboardAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void OnlineCalibrationKeyboardAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < OnlineCalibrationKeyboardAlgNode > (argc, argv, "online_calibration_keyboard_alg_node");
}

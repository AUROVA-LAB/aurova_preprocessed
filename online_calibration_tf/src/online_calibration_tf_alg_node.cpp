#include "online_calibration_tf_alg_node.h"

OnlineCalibrationTfAlgNode::OnlineCalibrationTfAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<OnlineCalibrationTfAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 20; //in [Hz]
  this->frame_id_ = "/velodyne"; //TODO: get from param
  this->child_frame_id_ = "/velodyne_calib"; //TODO: get from param
  this->transform_.header.frame_id = this->frame_id_;
  this->transform_.child_frame_id = this->child_frame_id_;
  this->transform_.header.stamp = ros::Time::now();
  this->transform_.transform.translation.x = 0.0;
  this->transform_.transform.translation.y = 0.0;
  this->transform_.transform.translation.z = 0.0;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
  this->transform_.transform.rotation.x = quaternion[0];
  this->transform_.transform.rotation.y = quaternion[1];
  this->transform_.transform.rotation.z = quaternion[2];
  this->transform_.transform.rotation.w = quaternion[3];

  // [init publishers]

  // [init subscribers]

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

OnlineCalibrationTfAlgNode::~OnlineCalibrationTfAlgNode(void)
{
  // [free dynamic memory]
}

void OnlineCalibrationTfAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->transform_.header.stamp = ros::Time::now();
  this->tf_broadcaster_.sendTransform(this->transform_);
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void OnlineCalibrationTfAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void OnlineCalibrationTfAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < OnlineCalibrationTfAlgNode > (argc, argv, "online_calibration_tf_alg_node");
}

#include "online_calibration_tf_alg_node.h"

OnlineCalibrationTfAlgNode::OnlineCalibrationTfAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<OnlineCalibrationTfAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 20; //in [Hz]
  this->frame_id_ = "/camera_link_static"; //TODO: get from param
  this->child_frame_id_ = "/camera_link"; //TODO: get from param
  this->transform_.header.frame_id = this->frame_id_;
  this->transform_.child_frame_id = this->child_frame_id_;
  this->transform_.header.stamp = ros::Time::now();

  // Open current_calibration file.
  // TODO: get path from param
  std::string current_tf;
  float x, y, z, qx, qy, qz, qw;
  this->current_tf_path_ =
      "/home/mice85/aurova-lab/aurova_ws/src/aurova_preprocessed/online_calibration_tf/data/current_tf.data";
  this->current_tf_file_i_.open(this->current_tf_path_.c_str());
  if (this->current_tf_file_i_.is_open())
  {
    while (std::getline(this->current_tf_file_i_, current_tf))
    {
      std::sscanf(current_tf.c_str(), " %f %f %f %f %f %f %f", &x, &y, &z, &qx, &qy, &qz, &qw);
      ROS_INFO("%s", current_tf.c_str());
    }
  }
  this->current_tf_file_i_.close();

  // TODO: get current values from file
  this->transform_.transform.translation.x = x;
  this->transform_.transform.translation.y = y;
  this->transform_.transform.translation.z = z;
  this->transform_.transform.rotation.x = qx;
  this->transform_.transform.rotation.y = qy;
  this->transform_.transform.rotation.z = qz;
  this->transform_.transform.rotation.w = qw;

  // [init publishers]

  // [init subscribers]
  this->delta_tf_subscriber_ = this->public_node_handle_.subscribe("/delta_tf", 1,
                                                                   &OnlineCalibrationTfAlgNode::cb_deltaTf, this);

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
void OnlineCalibrationTfAlgNode::cb_deltaTf(const geometry_msgs::Pose::ConstPtr& delta_tf)
{
  this->alg_.lock();

  double delta_roll, delta_pitch, delta_yaw;
  double current_roll, current_pitch, current_yaw;

  // get current transform
  tf::StampedTransform current_transform;
  try
  {
    tf_listener_.lookupTransform(this->frame_id_, this->child_frame_id_, ros::Time(0), current_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  // get RPY from current transform
  tf::Quaternion quaternion_aux(current_transform.getRotation().x(), current_transform.getRotation().y(),
                                current_transform.getRotation().z(), current_transform.getRotation().w());
  tf::Matrix3x3 matrix(quaternion_aux);
  matrix.getRPY(current_roll, current_pitch, current_yaw);

  // get RPY from delta_tf and integrate in new quaternion
  tf::Quaternion quaternion_aux2(delta_tf->orientation.x, delta_tf->orientation.y, delta_tf->orientation.z,
                                 delta_tf->orientation.w);
  tf::Matrix3x3 matrix2(quaternion_aux2);
  matrix2.getRPY(delta_roll, delta_pitch, delta_yaw);
  quaternion_aux = tf::createQuaternionFromRPY(current_roll + delta_roll, current_pitch + delta_pitch,
                                               current_yaw + delta_yaw);

  // apply delta_tf to new transform
  this->transform_.header.stamp = ros::Time::now();
  this->transform_.transform.translation.x = current_transform.getOrigin().x() + delta_tf->position.x;
  this->transform_.transform.translation.y = current_transform.getOrigin().y() + delta_tf->position.y;
  this->transform_.transform.translation.z = current_transform.getOrigin().z() + delta_tf->position.z;
  this->transform_.transform.rotation.x = quaternion_aux[0];
  this->transform_.transform.rotation.y = quaternion_aux[1];
  this->transform_.transform.rotation.z = quaternion_aux[2];
  this->transform_.transform.rotation.w = quaternion_aux[3];

  // save new current values to file
  std::ostringstream new_current_tf_o;
  std::string new_current_tf;
  this->current_tf_file_o_.open(this->current_tf_path_.c_str(), std::ofstream::trunc);
  new_current_tf_o << "   " << this->transform_.transform.translation.x << "   "
      << this->transform_.transform.translation.y << "   " << this->transform_.transform.translation.z << "   "
      << this->transform_.transform.rotation.x << "   " << this->transform_.transform.rotation.y << "   "
      << this->transform_.transform.rotation.z << "   " << this->transform_.transform.rotation.w << std::endl;
  new_current_tf = new_current_tf_o.str();
  this->current_tf_file_o_  << new_current_tf;
  this->current_tf_file_o_.close();

  this->alg_.unlock();
}

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

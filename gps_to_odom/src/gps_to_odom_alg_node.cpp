#include "gps_to_odom_alg_node.h"

GpsToOdomAlgNode::GpsToOdomAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<GpsToOdomAlgorithm>()
{
  //init class attributes if necessary
  this->flag_publish_odom_ = false;
  this->loop_rate_ = 10; //in [Hz]

  // [init publishers]
  this->odom_gps_pub_ = this->public_node_handle_.advertise < nav_msgs::Odometry > ("/odometry_gps", 1);

  // [init subscribers]
  this->odom_fix_sub_ = this->public_node_handle_.subscribe("/odometry_gps_fix", 1, &GpsToOdomAlgNode::cb_getGpsOdomMsg,
                                                            this);
  this->fix_vel_sub_ = this->public_node_handle_.subscribe("/rover/fix_velocity", 1,
                                                           &GpsToOdomAlgNode::cb_getGpsFixVelMsg, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

GpsToOdomAlgNode::~GpsToOdomAlgNode(void)
{
  // [free dynamic memory]
}

void GpsToOdomAlgNode::mainNodeThread(void)
{

  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  if (this->flag_publish_odom_)
  {
    this->odom_gps_pub_.publish(this->odom_gps_);
    this->flag_publish_odom_ = false;
  }
}

/*  [subscriber callbacks] */
void GpsToOdomAlgNode::cb_getGpsOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  this->alg_.lock();

  this->odom_gps_.header = odom_msg->header;
  this->odom_gps_.child_frame_id = odom_msg->child_frame_id;
  this->odom_gps_.pose.pose.position = odom_msg->pose.pose.position;
  this->odom_gps_.pose.covariance = odom_msg->pose.covariance;
  this->odom_gps_.pose.covariance[35] = 0.1;
  this->flag_publish_odom_ = true;

  this->alg_.unlock();
}

void GpsToOdomAlgNode::cb_getGpsFixVelMsg(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& vel_msg)
{
  this->alg_.lock();

  double vel_x, vel_y, vel_z, vel_abs;
  double orientation_yaw;

  vel_x = vel_msg->twist.twist.linear.x;
  vel_y = vel_msg->twist.twist.linear.y;
  vel_z = vel_msg->twist.twist.linear.z;

  vel_abs = sqrt(pow(vel_x, 2) + pow(vel_y, 2) + pow(vel_z, 2));

  //get transform
  try
  {
    this->listener_.lookupTransform("map", "utm", ros::Time(0), this->utm_trans_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  double yaw_tf = tf::getYaw(this->utm_trans_.getRotation());

  if (vel_abs > 0.5) //from parameter
  {
    orientation_yaw = acos(vel_x / vel_abs);

    if (vel_y < 0.0)
      orientation_yaw = -1 * orientation_yaw;

    orientation_yaw = orientation_yaw + yaw_tf;

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, orientation_yaw);

    this->odom_gps_.pose.pose.orientation.x = quaternion[0];
    this->odom_gps_.pose.pose.orientation.y = quaternion[1];
    this->odom_gps_.pose.pose.orientation.z = quaternion[2];
    this->odom_gps_.pose.pose.orientation.w = quaternion[3];

  }

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void GpsToOdomAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void GpsToOdomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < GpsToOdomAlgNode > (argc, argv, "gps_to_odom_alg_node");
}

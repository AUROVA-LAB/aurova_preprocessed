#include "gps_to_odom_alg_node.h"

GpsToOdomAlgNode::GpsToOdomAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<GpsToOdomAlgorithm>()
{
  //init class attributes if necessary
  this->flag_publish_odom_ = false;
  this->flag_gnss_position_received_ = false;
  this->flag_gnss_velocity_received_ = false;
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
  if (this->flag_gnss_position_received_ && this->flag_gnss_velocity_received_)
  {
    this->odom_gps_pub_.publish(this->odom_gps_);
    this->flag_gnss_position_received_ = false;
    this->flag_gnss_velocity_received_ = false;
  }
}

/*  [subscriber callbacks] */
void GpsToOdomAlgNode::cb_getGpsOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  this->alg_.lock();

  this->odom_gps_.header = odom_msg->header;
  this->odom_gps_.child_frame_id = odom_msg->child_frame_id;
  this->odom_gps_.pose.pose.position = odom_msg->pose.pose.position;
  this->odom_gps_.pose.covariance[0] = odom_msg->pose.covariance[0];
  this->odom_gps_.pose.covariance[7] = odom_msg->pose.covariance[7];
  this->odom_gps_.pose.covariance[14] = odom_msg->pose.covariance[14];
  this->flag_gnss_position_received_ = true;

  this->alg_.unlock();
}

double sign(double x)
{
  if (x > 0.0)
    return(+1.0);
  else
    return(-1.0);
}

void GpsToOdomAlgNode::cb_getGpsFixVelMsg(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& vel_msg)
{
  this->alg_.lock();

  double vx, vy, vz;

  vx = vel_msg->twist.twist.linear.x;
  vy = vel_msg->twist.twist.linear.y;
  vz = vel_msg->twist.twist.linear.z;

  // Passing the 3D velocities to the odometry twist in the output message
  // Mean value
  this->odom_gps_.twist.twist.linear.x = vx;
  this->odom_gps_.twist.twist.linear.y = vy;
  this->odom_gps_.twist.twist.linear.z = vz;
  // Linear velocities variances
  this->odom_gps_.twist.covariance = vel_msg->twist.covariance;
  this->odom_gps_.twist.covariance[21] = 0.0; // To correct some weird design that makes the ublox ros driver to
                                              // put a -1.0 value in this position


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

  double yaw_tf, pitch_tf, roll_tf;
  tf::Matrix3x3(this->utm_trans_.getRotation()).getRPY(roll_tf, pitch_tf,yaw_tf);

  if (sqrt(vx*vx + vy*vy + vz*vz) > 0.5) //from parameter
  {
    Eigen::Vector3d orientation_YPR = Eigen::Vector3d::Zero();
/*
    orientation_YPR(0) = sign(vy) * acos(vx / sqrt(vx*vx + vy*vy));
    orientation_YPR(1) = sign(vz) * acos(vx / sqrt(vx*vx + vz*vz));
    orientation_YPR(2) = sign(vz) * acos(vy / sqrt(vy*vy + vz*vz));
*/
    orientation_YPR(0) = atan2(vy, vx);
    orientation_YPR(1) = atan2(vz, vx);
    orientation_YPR(2) = atan2(vz, vy);
    // Converting from UTM to map frame
    orientation_YPR(0) = orientation_YPR(0) + yaw_tf;
    orientation_YPR(1) = orientation_YPR(1) + pitch_tf;
    orientation_YPR(2) = orientation_YPR(2) + roll_tf;

    // Converting to quarternion to fill the ROS message
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(orientation_YPR(2), orientation_YPR(1), orientation_YPR(0));
    this->odom_gps_.pose.pose.orientation.x = quaternion[0];
    this->odom_gps_.pose.pose.orientation.y = quaternion[1];
    this->odom_gps_.pose.pose.orientation.z = quaternion[2];
    this->odom_gps_.pose.pose.orientation.w = quaternion[3];

    // Calculate the orientation covariance matrix using a first order approximation:

    // Extract the velocity covariance
    Eigen::Matrix3d vel_covariance = Eigen::Matrix3d::Zero();
    vel_covariance(0,0) = vel_msg->twist.covariance[0];
    vel_covariance(1,1) = vel_msg->twist.covariance[7];
    vel_covariance(2,2) = vel_msg->twist.covariance[14];

    // Compute the jacobian of the function that maps velocities to orientation
    Eigen::Matrix3d orientation_YPR_jacobian = Eigen::Matrix3d::Zero();

    orientation_YPR_jacobian(0,0) = -1*fabs(vy) / (vx*vx + vy*vy); // partial derivative of yaw wrt vx
    orientation_YPR_jacobian(0,1) = sign(vy)*vx / (vx*vx + vy*vy); // partial derivative of yaw wrt vy
    orientation_YPR_jacobian(0,2) = 0.0;                           // partial derivative of yaw wrt vz

    orientation_YPR_jacobian(1,0) = -1*fabs(vz) / (vx*vx + vz*vz); // partial derivative of pitch wrt vx
    orientation_YPR_jacobian(1,1) = 0.0;                           // partial derivative of pitch wrt vy
    orientation_YPR_jacobian(1,2) = sign(vz)*vx / (vx*vx + vz*vz); // partial derivative of pitch wrt vz

    orientation_YPR_jacobian(2,0) = 0.0;                           // partial derivative of roll wrt vx
    orientation_YPR_jacobian(2,1) = -1*fabs(vz) / (vy*vy + vz*vz); // partial derivative of roll wrt vy
    orientation_YPR_jacobian(2,2) = sign(vz)*vy / (vy*vy + vz*vz); // partial derivative of roll wrt vz

    // Compute the first order approximation
    Eigen::Matrix3d orientation_YPR_cov = Eigen::Matrix3d::Zero();

    orientation_YPR_cov = orientation_YPR_jacobian * vel_covariance * orientation_YPR_jacobian.transpose();

    // Passing to ROS message
    this->odom_gps_.pose.covariance[21] = orientation_YPR_cov(0,0);
    this->odom_gps_.pose.covariance[22] = orientation_YPR_cov(0,1);
    this->odom_gps_.pose.covariance[23] = orientation_YPR_cov(0,2);

    this->odom_gps_.pose.covariance[27] = orientation_YPR_cov(1,0);
    this->odom_gps_.pose.covariance[28] = orientation_YPR_cov(1,1);
    this->odom_gps_.pose.covariance[29] = orientation_YPR_cov(1,2);

    this->odom_gps_.pose.covariance[33] = orientation_YPR_cov(2,0);
    this->odom_gps_.pose.covariance[34] = orientation_YPR_cov(2,1);
    this->odom_gps_.pose.covariance[35] = orientation_YPR_cov(2,2);

  }

  this->flag_gnss_velocity_received_ = true;
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

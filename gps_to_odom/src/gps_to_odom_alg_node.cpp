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
  this->odom_gps_.pose.pose.position = odom_msg->pose.pose.position; // In Map frame
  this->odom_gps_.pose.covariance[0] = odom_msg->pose.covariance[0];
  this->odom_gps_.pose.covariance[7] = odom_msg->pose.covariance[7];
  this->odom_gps_.pose.covariance[14] = odom_msg->pose.covariance[14];
  this->flag_gnss_position_received_ = true;

  this->alg_.unlock();
}

void GpsToOdomAlgNode::cb_getGpsFixVelMsg(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& vel_msg)
{
  this->alg_.lock();

  // We start working in the original UTM coordinates, but we will convert the data to Map frame prior to publish them

  // We get the velocities from the message
  Eigen::Vector3d UTM_velocities = Eigen::Vector3d::Zero();
  UTM_velocities(0) = vel_msg->twist.twist.linear.x; // In UTM frame
  UTM_velocities(1) = vel_msg->twist.twist.linear.y;
  UTM_velocities(2) = vel_msg->twist.twist.linear.z;

  double vx = UTM_velocities(0); // just to improve readability
  double vy = UTM_velocities(1);
  double vz = UTM_velocities(2);

  // And compute the orientation as a function of the velocities
  Eigen::Vector3d UTM_orientation_YPR = Eigen::Vector3d::Zero();
  UTM_orientation_YPR(0) = atan2(vy, vx);
  UTM_orientation_YPR(1) = atan2(vz, vx);
  UTM_orientation_YPR(2) = atan2(vz, vy);


  // Now we need to compute the covariance matrix of those orientations,
  // as we have an explicit non-linear relation, we use the a first order approximation
  // to describe the variance propagation:

  // First, we compute the jacobian of the function that maps velocities to orientation
  Eigen::Matrix3d UTM_orientation_YPR_jacobian = Eigen::Matrix3d::Zero();

  UTM_orientation_YPR_jacobian(0,0) = -1*vy / (vx*vx + vy*vy); // partial derivative of yaw wrt vx
  UTM_orientation_YPR_jacobian(0,1) =    vx / (vx*vx + vy*vy); // partial derivative of yaw wrt vy
  UTM_orientation_YPR_jacobian(0,2) = 0.0;                     // partial derivative of yaw wrt vz

  UTM_orientation_YPR_jacobian(1,0) = -1*vz / (vx*vx + vz*vz); // partial derivative of pitch wrt vx
  UTM_orientation_YPR_jacobian(1,1) = 0.0;                     // partial derivative of pitch wrt vy
  UTM_orientation_YPR_jacobian(1,2) =    vx / (vx*vx + vz*vz); // partial derivative of pitch wrt vz

  UTM_orientation_YPR_jacobian(2,0) = 0.0;                     // partial derivative of roll wrt vx
  UTM_orientation_YPR_jacobian(2,1) = -1*vz / (vy*vy + vz*vz); // partial derivative of roll wrt vy
  UTM_orientation_YPR_jacobian(2,2) =    vy / (vy*vy + vz*vz); // partial derivative of roll wrt vz


  // Next, we extract the velocity covariance matrix (it will be just diagonal using an UBLOX M8P sensor)
  Eigen::Matrix3d UTM_vel_covariance = Eigen::Matrix3d::Zero();
  UTM_vel_covariance(0,0) = vel_msg->twist.covariance[0];
  UTM_vel_covariance(0,1) = vel_msg->twist.covariance[1];
  UTM_vel_covariance(0,2) = vel_msg->twist.covariance[2];

  UTM_vel_covariance(1,0) = vel_msg->twist.covariance[6];
  UTM_vel_covariance(1,1) = vel_msg->twist.covariance[7];
  UTM_vel_covariance(1,2) = vel_msg->twist.covariance[8];

  UTM_vel_covariance(2,0) = vel_msg->twist.covariance[12];
  UTM_vel_covariance(2,1) = vel_msg->twist.covariance[13];
  UTM_vel_covariance(2,2) = vel_msg->twist.covariance[14];

  // Finally, we compute the first order approximation
  Eigen::Matrix3d UTM_orientation_YPR_cov = Eigen::Matrix3d::Zero();

  UTM_orientation_YPR_cov = UTM_orientation_YPR_jacobian * UTM_vel_covariance * UTM_orientation_YPR_jacobian.transpose();


  // Now we need to transform all the data from UTM to Map frame

  // First, we get transform UTM to MAP
  try
  {
    this->listener_.lookupTransform("map", "utm", ros::Time(0), this->utm_trans_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  // Convert it to Eigen
  Eigen::Affine3d UTM_to_map_rotation;
  tf::transformTFToEigen(this->utm_trans_, UTM_to_map_rotation);

  // Next, we convert the velocities
  Eigen::Vector3d map_velocities = Eigen::Vector3d::Zero();
  map_velocities = UTM_to_map_rotation.linear() * UTM_velocities;

  // And the orientations
  Eigen::Vector3d map_orientations_YPR = Eigen::Vector3d::Zero();
  map_orientations_YPR = UTM_to_map_rotation.linear() * UTM_orientation_YPR;

  // Converting to quarternion to fill the ROS message
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(map_orientations_YPR(2), map_orientations_YPR(1), map_orientations_YPR(0));
  this->odom_gps_.pose.pose.orientation.x = quaternion[0];
  this->odom_gps_.pose.pose.orientation.y = quaternion[1];
  this->odom_gps_.pose.pose.orientation.z = quaternion[2];
  this->odom_gps_.pose.pose.orientation.w = quaternion[3];

  // And finally the covariances
  Eigen::Matrix3d map_vel_cov = Eigen::Matrix3d::Zero();
  map_vel_cov = UTM_to_map_rotation.linear() * UTM_vel_covariance * UTM_to_map_rotation.linear().transpose();

  Eigen::Matrix3d map_orientations_YPR_cov = Eigen::Matrix3d::Zero();
  map_orientations_YPR_cov = UTM_to_map_rotation.linear() * UTM_orientation_YPR_cov * UTM_to_map_rotation.linear().transpose();

  // Passing to ROS message
  this->odom_gps_.pose.covariance[21] = map_orientations_YPR_cov(0,0);
  this->odom_gps_.pose.covariance[22] = map_orientations_YPR_cov(0,1);
  this->odom_gps_.pose.covariance[23] = map_orientations_YPR_cov(0,2);

  this->odom_gps_.pose.covariance[27] = map_orientations_YPR_cov(1,0);
  this->odom_gps_.pose.covariance[28] = map_orientations_YPR_cov(1,1);
  this->odom_gps_.pose.covariance[29] = map_orientations_YPR_cov(1,2);

  this->odom_gps_.pose.covariance[33] = map_orientations_YPR_cov(2,0);
  this->odom_gps_.pose.covariance[34] = map_orientations_YPR_cov(2,1);
  this->odom_gps_.pose.covariance[35] = map_orientations_YPR_cov(2,2);

  // Passing the 3D velocities to the odometry twist in the output message
  // Mean value
  this->odom_gps_.twist.twist.linear.x = map_velocities(0);
  this->odom_gps_.twist.twist.linear.y = map_velocities(1);
  this->odom_gps_.twist.twist.linear.z = map_velocities(2);

  // Linear velocities covariance matrix
  this->odom_gps_.twist.covariance[0]  = map_vel_cov(0,0);
  this->odom_gps_.twist.covariance[1]  = map_vel_cov(0,1);
  this->odom_gps_.twist.covariance[2]  = map_vel_cov(0,2);

  this->odom_gps_.twist.covariance[6]  = map_vel_cov(1,0);
  this->odom_gps_.twist.covariance[7]  = map_vel_cov(1,1);
  this->odom_gps_.twist.covariance[8]  = map_vel_cov(1,2);

  this->odom_gps_.twist.covariance[12] = map_vel_cov(2,0);
  this->odom_gps_.twist.covariance[13] = map_vel_cov(2,1);
  this->odom_gps_.twist.covariance[14] = map_vel_cov(2,2);

  this->odom_gps_.twist.covariance[21] = 0.0; // To correct some weird design that makes the ublox ros driver to
                                              // put a -1.0 value in this position
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

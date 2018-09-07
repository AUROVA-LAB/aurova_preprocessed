#include "ackermann_to_odom_alg.h"
#include <time.h>

AckermannToOdomAlgorithm::AckermannToOdomAlgorithm(void)
{
  pthread_mutex_init(&this->access_, NULL);
}

AckermannToOdomAlgorithm::~AckermannToOdomAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void AckermannToOdomAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

// AckermannToOdomAlgorithm Public API
void AckermannToOdomAlgorithm::generateNewOdometryMsg(ackermann_msgs::AckermannDriveStamped estimated_ackermann_state,
                                                      double covariance, sensor_msgs::Imu virtual_imu_msg,
                                                      nav_msgs::Odometry& odometry, geometry_msgs::TransformStamped& odom_trans,
                                                      geometry_msgs::TransformStamped& base_trans)
{

  const float WHEELBASE_METERS = 1.05;
  const int ROWS = 6;
  const int COLUMNS = 6;

  int i, j;
  float orientation_z = 0, orientation_z_prev = 0;
  float pose_x_prev = 0;
  float pose_y_prev = 0;
  static double t_1;
  static double t_2;
  static double first_exec = 1;

  if (covariance == 0.0)
    covariance = 0.01;

  /////////////////////////////////////////////////
  //// POSE AND VELOCITY
  //calculate increment of time
  if (first_exec)
  {
    t_2 = (double)ros::Time::now().toSec();
    first_exec = 0;
  }
  t_1 = (double)ros::Time::now().toSec();
  float delta_t = (float)(t_1 - t_2);
  t_2 = (double)ros::Time::now().toSec();
  float lineal_speed = estimated_ackermann_state.drive.speed;

  //angle
  tf::Quaternion q(virtual_imu_msg.orientation.x, virtual_imu_msg.orientation.y, virtual_imu_msg.orientation.z,
                   virtual_imu_msg.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  orientation_z = yaw;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, orientation_z);
  /*
   double steering_radians = estimated_ackermann_state.drive.steering_angle * M_PI / 180.0;
   float angular_speed_z   = (lineal_speed / WHEELBASE_METERS) * tan(steering_radians);
   orientation_z           = orientation_z_prev + angular_speed_z*delta_t;
   if (abs(orientation_z) >= (2.0 * M_PI))
   orientation_z = orientation_z - (2.0 * M_PI);
   tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, orientation_z);
   */

  //pose
  float lineal_speed_x = lineal_speed * cos(orientation_z);
  float lineal_speed_y = lineal_speed * sin(orientation_z);
  float pose_x = pose_x_prev + lineal_speed_x * delta_t;
  float pose_y = pose_y_prev + lineal_speed_y * delta_t;
  if (isnan(orientation_z))
  {
    lineal_speed_x = 0.0;
    lineal_speed_y = 0.0;
    pose_x = 0.0;
    pose_y = 0.0;
    quaternion = tf::createQuaternionFromRPY(0, 0, 0);
  }

  // For next step
  //orientation_z_prev = orientation_z;
  pose_x_prev = pose_x;
  pose_y_prev = pose_y;
  /////////////////////////////////////////////////

  /////////////////////////////////////////////////
  //// GENERATE MESSAGE
  // Header
  odometry.header.stamp = ros::Time::now();
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_link";

  // Twist
  odometry.twist.twist.linear.x = lineal_speed_x;
  odometry.twist.twist.linear.y = lineal_speed_y;
  odometry.twist.twist.linear.z = 0;
  odometry.twist.twist.angular.x = 0;
  odometry.twist.twist.angular.y = 0;
  odometry.twist.twist.angular.z = 0;
  for (i = 0; i < COLUMNS; i++)
  {
    odometry.twist.covariance[i * ROWS + i] = covariance;
  }

  // Pose
  odometry.pose.pose.position.x = pose_x;
  odometry.pose.pose.position.y = pose_y;
  odometry.pose.pose.position.z = 0;
  odometry.pose.pose.orientation.x = quaternion[0];
  odometry.pose.pose.orientation.y = quaternion[1];
  odometry.pose.pose.orientation.z = quaternion[2];
  odometry.pose.pose.orientation.w = quaternion[3];
  for (i = 0; i < COLUMNS; i++)
  {
    odometry.pose.covariance[i * ROWS + i] = covariance;
  }
  /////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////
  //// GENERATE MESSAGES TF
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = pose_x;
  odom_trans.transform.translation.y = pose_y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(orientation_z);

  base_trans.header.frame_id = "base_link";
  base_trans.child_frame_id = "velodyne";
  base_trans.header.stamp = ros::Time::now();
  base_trans.transform.translation.x = 0.55;
  base_trans.transform.translation.y = 0.0;
  base_trans.transform.translation.z = 0.0;
  base_trans.transform.rotation = tf::createQuaternionMsgFromYaw(-0.005);
  ////////////////////////////////////////////////////////////////
}

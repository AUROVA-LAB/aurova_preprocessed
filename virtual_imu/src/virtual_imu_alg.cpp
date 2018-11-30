#include "virtual_imu_alg.h"

VirtualImuAlgorithm::VirtualImuAlgorithm(void)
{
  this->estimation_rpy_ = new KalmanFilter();

  pthread_mutex_init(&this->access_, NULL);
}

VirtualImuAlgorithm::~VirtualImuAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void VirtualImuAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

// VirtualImuAlgorithm Public API
void VirtualImuAlgorithm::createVirtualImu(sensor_msgs::Imu originl_imu_msg, sensor_msgs::Imu& virtual_imu_msg)
{

  static double t_1;
  static double t_2;
  static double first_exec = 1;

  float delta_t;

  //calculate delta time
  if (first_exec)
  {
    t_2 = (double)ros::Time::now().toSec();
    first_exec = 0;
  }
  t_1 = (double)ros::Time::now().toSec();
  delta_t = (float)(t_1 - t_2);
  t_2 = (double)ros::Time::now().toSec();

  //orientation calculations
  this->estimation_rpy_->predict(delta_t, originl_imu_msg.angular_velocity.x, originl_imu_msg.angular_velocity.y,
                                 originl_imu_msg.angular_velocity.z);
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(this->estimation_rpy_->X_[0][0],
                                                          this->estimation_rpy_->X_[1][0],
                                                          this->estimation_rpy_->X_[2][0]);

  //create message
  virtual_imu_msg.header.stamp = ros::Time::now();
  virtual_imu_msg.header.frame_id = "imu_link";
  virtual_imu_msg.orientation.x = quaternion[0];
  virtual_imu_msg.orientation.y = quaternion[1];
  virtual_imu_msg.orientation.z = quaternion[2];
  virtual_imu_msg.orientation.w = quaternion[3];
  virtual_imu_msg.orientation_covariance[0] = 0.5;
  virtual_imu_msg.orientation_covariance[4] = 0.5;
  virtual_imu_msg.orientation_covariance[8] = 0.5;
}

void VirtualImuAlgorithm::rpyFromGpsVelocity(float& roll, float& pitch, float& yaw, float x, float y, float z)
{

}

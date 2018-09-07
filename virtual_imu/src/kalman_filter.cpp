/*
 * kalman_filter.cpp
 *
 *  Created on: 12 Jul 2018
 *      Author: m.a.munoz
 */

#include "../include/kalman_filter.h"

KalmanFilter::KalmanFilter(void)
{
  //Initializations:

  //State
  X_[0][0] = 0.0; //Roll
  X_[1][0] = 0.0; //Pitch
  X_[2][0] = 0.0; //Yaw

  //Covariance matrix
  P_[0][0] = 0.0;
  P_[0][1] = 0.0;
  P_[0][2] = 0.0;

  // [...]
}

void KalmanFilter::predict(float delta_t, float roll_rate, float pitch_rate, float yaw_rate)
{
  //State prediction
  X_[0][0] = X_[0][0] - delta_t * roll_rate;
  X_[1][0] = X_[1][0] - delta_t * pitch_rate;
  X_[2][0] = X_[2][0] - delta_t * yaw_rate;

  // [...]
}

void KalmanFilter::correct(float roll_obs, float pitch_obs, float yaw_obs)
{
  //Correction step
  if (yaw_obs > MIN_CODE*0.8)
  {
    X_[0][0] = roll_obs;
    X_[1][0] = pitch_obs;
    X_[2][0] = yaw_obs;

    //debug
    //ROS_INFO("yaw: [%f]", yaw_obs);
  }

}

void KalmanFilter::getState(void)
{
  // [...]
}

void KalmanFilter::getVariances(void)
{
  // [...]
}


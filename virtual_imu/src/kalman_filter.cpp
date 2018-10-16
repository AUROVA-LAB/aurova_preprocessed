#include "kalman_filter.h"

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

  if (abs(delta_t * roll_rate) < MAX_DIFF && abs(delta_t * pitch_rate) < MAX_DIFF && abs(delta_t * yaw_rate) < MAX_DIFF)
  {
    X_[0][0] = X_[0][0] - delta_t * roll_rate;
    X_[1][0] = X_[1][0] - delta_t * pitch_rate;
    X_[2][0] = X_[2][0] - delta_t * yaw_rate;
  }
  else
  {
    X_[0][0] = X_[0][0];
    X_[1][0] = X_[1][0];
    X_[2][0] = X_[2][0];
  }

  // [...]
}

void KalmanFilter::correct(float roll_obs, float pitch_obs, float yaw_obs)
{
  // [...]
}

void KalmanFilter::getState(void)
{
  // [...]
}

void KalmanFilter::getVariances(void)
{
  // [...]
}


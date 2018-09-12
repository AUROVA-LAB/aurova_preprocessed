/**
 * \file kalman_filter.h
 *
 *  Created on: 12 Jul 2018
 *      Author: m.a.munoz
 */

#ifndef _kalman_filter_h_
#define _kalman_filter_h_

#include "math.h"
#include "ros/ros.h"
//#include "../libraries/MatrixMath-master/MatrixMath.h" // utilizar eigen

#define MIN_SPEED 0.5
#define MIN_CODE -77
#define GUARD 0.8

class KalmanFilter;
typedef KalmanFilter* KalmanFilterPtr;

/**
 * \brief Specific Kalman Filter
 *
 * Kalman Filter methods for estimate angle position
 */
class KalmanFilter
{
private:

public:

  //state vector of kalman filter process
  float X_[3][1];
  //covariance vector of kalman filter process
  float P_[3][3];

  /**
   * \brief Constructor of KalmanFilter class
   *
   * Inicialization of the process, state n = 0.
   */
  KalmanFilter(void);

  /**
   * Prediction step for kalman filter with model of rpy rate integration
   */
  void predict(float delta_t, float roll_rate, float pitch_rate, float yaw_rate);

  /**
   * Correction step for kalman filter with the angular position calculate with gps
   */
  void correct(float roll_obs, float pitch_obs, float yaw_obs);

  void getState(void);

  void getVariances(void);
};

#endif /* _kalman_filter_h_ */

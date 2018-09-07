/*
 * kalman_filter.h
 *
 *  Created on: 12 Jul 2018
 *      Author: m.a.munoz
 */

#ifndef HEADERS_KALMAN_FILTER_H_
#define HEADERS_KALMAN_FILTER_H_

#include "math.h"
#include "ros/ros.h"
//#include "../libraries/MatrixMath-master/MatrixMath.h" // utilizar eigen

#define MIN_SPEED 0.5
#define MIN_CODE -77


class KalmanFilter;
typedef  KalmanFilter* KalmanFilterPtr;

class KalmanFilter
{
private:

public:
	float X_[3][1];
	float P_[3][3];

	KalmanFilter(void);

	void predict(float delta_t, float roll_rate, float pitch_rate, float yaw_rate);

	void correct(float roll_obs, float pitch_obs, float yaw_obs);

	void getState(void);

	void getVariances(void);
};


#endif /* HEADERS_KALMAN_FILTER_H_ */

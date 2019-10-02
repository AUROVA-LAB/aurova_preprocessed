// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab
/**
 * \file online_calibration_alg.h
 *
 *  Created on: 29 May 2019
 *      Author: m.a.munoz
 */

#ifndef _online_calibration_alg_h_
#define _online_calibration_alg_h_

#include <online_calibration/OnlineCalibrationConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <termios.h>
#include <map>

//include online_calibration_alg main library

#define INVALID_VALUE -1
#define NO_INDEX -2
#define EMPTY_PIXEL 0.0
#define MAX_PIXEL 255.0

struct SensorConfiguration
{
  float max_elevation_angle;
  float min_elevation_angle;

  float max_azimuth_angle;
  float min_azimuth_angle;

  float grid_azimuth_angular_resolution;
  float grid_elevation_angular_resolution;

  int num_of_azimuth_cells; // To calculate these values: 1 + (max_azimuth_angle - min_azimuth_angle) / grid_azimuth_angular_resolution;
  int num_of_elevation_cells; // or 1 + (max_elevation_angle - min_elevation_angle) / grid_elevation_angular_resolution;

  float max_range;
  float sensor_height;
};

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class OnlineCalibrationAlgorithm
{
protected:
  /**
   * \brief define config type
   *
   * Define a Config type with the OnlineCalibrationConfig. All driver implementations
   * will then use the same variable type Config.
   */
  pthread_mutex_t access_;

  // private attributes and methods

public:

  /**
   * \brief define config type
   *
   * Define a Config type with the OnlineCalibrationConfig. All driver implementations
   * will then use the same variable type Config.
   */
  typedef online_calibration::OnlineCalibrationConfig Config;
  struct SensorConfiguration st_sens_config_;

  /**
   * \brief config variable
   *
   * This variable has all the driver parameters defined in the cfg config file.
   * Is updated everytime function config_update() is called.
   */
  Config config_;

  /**
   * \brief constructor
   *
   * In this constructor parameters related to the specific driver can be
   * initalized. Those parameters can be also set in the openDriver() function.
   * Attributes from the main node driver class IriBaseDriver such as loop_rate,
   * may be also overload here.
   */
  OnlineCalibrationAlgorithm(void);

  /**
   * \brief Lock Algorithm
   *
   * Locks access to the Algorithm class
   */
  void lock(void)
  {
    pthread_mutex_lock(&this->access_);
  }
  ;

  /**
   * \brief Unlock Algorithm
   *
   * Unlocks access to the Algorithm class
   */
  void unlock(void)
  {
    pthread_mutex_unlock(&this->access_);
  }
  ;

  /**
   * \brief Tries Access to Algorithm
   *
   * Tries access to Algorithm
   *
   * \return true if the lock was adquired, false otherwise
   */
  bool try_enter(void)
  {
    if (pthread_mutex_trylock(&this->access_) == 0)
      return true;
    else
      return false;
  }
  ;

  /**
   * \brief config update
   *
   * In this function the driver parameters must be updated with the input
   * config variable. Then the new configuration state will be stored in the
   * Config attribute.
   *
   * \param new_cfg the new driver configuration state
   *
   * \param level level in which the update is taken place
   */
  void config_update(Config& config, uint32_t level = 0);

  // here define all online_calibration_alg interface methods to retrieve and set
  // the driver parameters

  /**
   * \brief Destructor
   *
   * This destructor is called when the object is about to be destroyed.
   *
   */
  ~OnlineCalibrationAlgorithm(void);

  /**
   * TODO: doxygen comments
   */
  void filterSensorsData(cv::Mat last_image, sensor_msgs::PointCloud2 scan,
                         image_geometry::PinholeCameraModel cam_model, std::string frame_lidar,
                         ros::Time acquisition_time, tf::TransformListener& tf_listener,
                         sensor_msgs::PointCloud2& scan_discontinuities, cv::Mat& plot_image, cv::Mat& image_sobel,
                         cv::Mat& image_sobel_plot);

  /**
   * \brief Method that ...
   *
   * This method gets as principal inputs:
   * @param last_image: is an openCV image.
   * @param scan: is a ...
   *
   * And return as outputs:
   * @param image_discontinuities: is the ...
   */
  void acumAndProjectPoints(cv::Mat last_image, sensor_msgs::PointCloud2 scan,
                            image_geometry::PinholeCameraModel cam_model, std::string frame_lidar,
                            std::string frame_odom, ros::Time acquisition_time, tf::TransformListener& tf_listener,
                            cv::Mat& image_sobel_plot, cv::Mat& image_discontinuities);

  /**
   * TODO: doxygen comments
   */
  void featureMatching(cv::Mat& depth_map, cv::Mat& color_map, cv::Mat& image_matches);
};

#endif

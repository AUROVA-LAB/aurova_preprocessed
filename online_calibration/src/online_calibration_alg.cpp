#include "online_calibration_alg.h"

void cartesian2SphericalInDegrees(float x, float y, float z, float& range, float& azimuth, float& elevation);
void point2SphericalGrid(pcl::PointXYZI point, struct SensorConfiguration lidar_configuration, int& row, int& col);
float colorMap(float dist, float factor, int base);
float colorMapExp(float dist, float sigma, int base);

OnlineCalibrationAlgorithm::OnlineCalibrationAlgorithm(void)
{
  pthread_mutex_init(&this->access_, NULL);
}

OnlineCalibrationAlgorithm::~OnlineCalibrationAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void OnlineCalibrationAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

// OnlineCalibrationAlgorithm Public API
void OnlineCalibrationAlgorithm::filterSensorsData(cv::Mat last_image, sensor_msgs::PointCloud2 scan,
                                                   image_geometry::PinholeCameraModel cam_model,
                                                   std::string frame_lidar, ros::Time acquisition_time,
                                                   tf::TransformListener& tf_listener,
                                                   sensor_msgs::PointCloud2& scan_discontinuities, cv::Mat& plot_image,
                                                   cv::Mat& image_sobel, cv::Mat& image_sobel_plot)
{
  /****** variable declarations ******/
  int i, j, k;
  int rows = last_image.rows;
  int cols = last_image.cols;
  float factor_color = 80.0; //TODO: get from parameter (only for plot)
  sensor_msgs::PointCloud2 scan_transformed;
  pcl::PCLPointCloud2 scan_pcl2;
  static pcl::PointCloud<pcl::PointXYZI> scan_pcl;
  static pcl::PointCloud<pcl::PointXYZI> scan_transformed_pcl;
  int index[rows][cols];
  for (i = 0; i < cols; i++)
    for (j = 0; j < rows; j++)
      index[j][i] = NO_INDEX;

  /***********conversion to grayscale **********/
  //cv::Mat last_image_gray;
  //cv::cvtColor(last_image, last_image_gray, CV_BGR2GRAY);
  //cv::cvtColor(last_image_gray, image_sobel, cv::COLOR_GRAY2BGR);
  //cv::cvtColor(last_image_gray, image_sobel_plot, cv::COLOR_GRAY2BGR);
  //cv::applyColorMap(image_sobel, image_sobel, cv::COLORMAP_JET);

  /************** sobel filter **************/
  /*cv::Mat last_image_gray_filter;
  cv::Mat grad;
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

  cv::GaussianBlur(last_image, last_image, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
  cv::cvtColor(last_image, last_image_gray_filter, CV_BGR2GRAY);
  cv::Sobel(last_image_gray_filter, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  cv::Sobel(last_image_gray_filter, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs(grad_x, abs_grad_x);
  convertScaleAbs(grad_y, abs_grad_y);
  addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
  grad.copyTo(image_sobel);
  cv::cvtColor(image_sobel, image_sobel_plot, cv::COLOR_GRAY2BGR);
  cv::cvtColor(image_sobel, image_sobel, cv::COLOR_GRAY2BGR);*/

  /************* canny detector *****************/
  //cv::Mat detected_edges;
  //int lowThreshold = 20;
  //int const max_lowThreshold = 100;
  //int ratio = 3;
  //int kernel_size = 3;
  //cv::GaussianBlur(last_image, last_image, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
  //cv::cvtColor(last_image, last_image_gray, CV_BGR2GRAY);
  //cv::blur(last_image_gray, detected_edges, cv::Size(3, 3));
  //cv::Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio, kernel_size);
  ////image_sobel = cv::Scalar::all(0);
  ////last_image.copyTo(image_sobel, detected_edges);
  //detected_edges.copyTo(image_sobel);


  /************** transform scan to camera frame ****************/
  try
  {
    ros::Duration duration(5.0);
    tf_listener.waitForTransform(cam_model.tfFrame(), frame_lidar, ros::Time::now(), duration);
    pcl_ros::transformPointCloud(cam_model.tfFrame(), scan, scan_transformed, tf_listener);

  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }

  /**************  conversions from msg to pcl format ****************/
  pcl_conversions::toPCL(scan, scan_pcl2);
  pcl::fromPCLPointCloud2(scan_pcl2, scan_pcl);
  pcl_conversions::toPCL(scan_transformed, scan_pcl2);
  pcl::fromPCLPointCloud2(scan_pcl2, scan_transformed_pcl);

  /****** get field of view and save index correspondence between cloud and image ******/
  /*float max_elevation = -10000.0;
  float min_elevation = 10000.0;
  float max_azimut = -10000.0;
  float min_azimut = 10000.0;*/
  static pcl::PointCloud<pcl::PointXYZI> scan_discontinuities_pcl;
  scan_discontinuities_pcl.clear(); // because is static
  for (i = 0; i < scan_transformed_pcl.points.size(); ++i)
  {
    if (scan_transformed_pcl.points[i].z > 0.0)
    {
      // project into image plane
      cv::Point3d pt_cv(scan_transformed_pcl.points[i].x, scan_transformed_pcl.points[i].y,
                        scan_transformed_pcl.points[i].z);
      cv::Point2d uv;
      uv = cam_model.project3dToPixel(pt_cv);
      if (uv.x >= 0 && uv.y >= 0 && uv.x < cols && uv.y < rows)
      {

        // Extract espherical coordinates and save max/min
        /*float range = 0.0;
        float elevation = 0.0;
        float azimuth = 0.0;
        float x = scan_pcl.points[i].x;
        float y = scan_pcl.points[i].y;
        float z = scan_pcl.points[i].z;
        cartesian2SphericalInDegrees(x, y, z, range, azimuth, elevation);

        if (azimuth > 180.0)
        {
          azimuth = azimuth - 360.0;
        }
        if (azimuth > max_azimut)
          max_azimut = azimuth;
        if (azimuth < min_azimut)
          min_azimut = azimuth;
        if (elevation > max_elevation)
          max_elevation = elevation;
        if (elevation < min_elevation)
          min_elevation = elevation;*/

        //// Save (x, y, z) and (u, v) information
        index[(int)uv.y][(int)uv.x] = i;

        // plot points in image
        static const int RADIUS = 1;
        float r = 0.0;
        float g = colorMap(scan_transformed_pcl.points[i].z, factor_color, MAX_PIXEL);
        float b = 0.0;
        cv::circle(plot_image, uv, RADIUS, CV_RGB(r, g, b), -1);

        scan_discontinuities_pcl.push_back(scan_pcl.points[i]);
      }
    }
  }

  /********* fill the sensor structure *********/
  /*this->st_sens_config_.max_elevation_angle = max_elevation;
  this->st_sens_config_.min_elevation_angle = min_elevation;
  this->st_sens_config_.max_azimuth_angle = max_azimut;
  this->st_sens_config_.min_azimuth_angle = min_azimut;
  this->st_sens_config_.max_range = 100.0; //// TODO: from parameter
  this->st_sens_config_.grid_azimuth_angular_resolution = 0.2; //// TODO: from parameter
  this->st_sens_config_.grid_elevation_angular_resolution = 2.0; //// TODO: from parameter
  this->st_sens_config_.num_of_azimuth_cells = 1
      + (max_azimut - min_azimut) / this->st_sens_config_.grid_azimuth_angular_resolution;
  this->st_sens_config_.num_of_elevation_cells = 1
      + (max_elevation - min_elevation) / this->st_sens_config_.grid_elevation_angular_resolution;*/

  /********* generate ordened cloud divided in scan slices *********/
  /*int num_slices = this->st_sens_config_.num_of_elevation_cells;
  static std::vector<pcl::PointCloud<pcl::PointXYZI> > scan_slices;
  //static std::vector<pcl::PointCloud<pcl::PointXYZI> > gray_slices;
  scan_slices.clear();
  //gray_slices.clear();
  scan_slices.resize(num_slices);
  //gray_slices.resize(num_slices);
  int u = 0, v = 0;
  for (i = 0; i < cols; i++)
  {
    for (j = 0; j < rows; j++)
    {
      if (index[j][i] > NO_INDEX)
      {
        k = index[j][i];
        point2SphericalGrid(scan_pcl.points[k], this->st_sens_config_, v, u);
        if (u != INVALID_VALUE)
        {
          scan_slices[v].push_back(scan_pcl.points[k]);
        }
      }
    }
  }*/

  /************** filter slices of scan *********************/
  /*float range_pr = 0.0;
  float range_ps = 0.0;
  float range_ac = 0.0;
  float azimuth_pr = 0.0;
  float azimuth_ps = 0.0;
  float azimuth_ac = 0.0;
  float elevation = 0.0;
  float alpha = 1.0; //// TODO: from parameter
  float threshold = 0.0; //// TODO: from parameter
  float threshold_up = 50.0; //// TODO: from parameter
  float max_range = 10.0; //// TODO: from parameter
  static pcl::PointCloud<pcl::PointXYZI> scan_discontinuities_pcl;
  scan_discontinuities_pcl.clear(); // because is static
  for (i = 0; i < scan_slices.size(); i++)
  {
    for (j = 0; j < scan_slices[i].size(); j++)
    {
      // actual points
      cartesian2SphericalInDegrees(scan_slices[i].points[j].x, scan_slices[i].points[j].y, scan_slices[i].points[j].z,
                                   range_ac, azimuth_ac, elevation);
      if (azimuth_ac > 180.0) // azimut correction
      {
        azimuth_ac = azimuth_ac - 360.0;
      }

      // previous points
      if (j - 1 < 0)
      {
        range_pr = range_ac;
      }
      else
      {
        cartesian2SphericalInDegrees(scan_slices[i].points[j - 1].x, scan_slices[i].points[j - 1].y,
                                     scan_slices[i].points[j - 1].z, range_pr, azimuth_pr, elevation);
        if (azimuth_pr > 180.0) // azimut correction
        {
          azimuth_pr = azimuth_pr - 360.0;
        }

        if (abs(azimuth_ac - azimuth_pr) > this->st_sens_config_.grid_azimuth_angular_resolution * 2)
        {
          range_pr = this->st_sens_config_.max_range;
        }
      }

      // posterior points
      if (j + 1 >= scan_slices[i].size())
      {
        range_ps = range_ac;
      }
      else
      {
        cartesian2SphericalInDegrees(scan_slices[i].points[j + 1].x, scan_slices[i].points[j + 1].y,
                                     scan_slices[i].points[j + 1].z, range_ps, azimuth_ps, elevation);
        if (azimuth_ps > 180.0) // azimut correction
        {
          azimuth_ps = azimuth_ps - 360.0;
        }

        if (abs(azimuth_ac - azimuth_ps) > this->st_sens_config_.grid_azimuth_angular_resolution * 2)
        {
          range_ps = this->st_sens_config_.max_range;
        }
      }

      // filter points
      float max_range = std::max(range_pr - range_ac, range_ps - range_ac);
      float max_final = std::max(max_range, (float)0.0);
      max_final = pow(max_final, alpha);
      if (max_final > threshold && range_ac < max_range)
      {
        float scope = max_final - threshold;
        float scope_max = threshold_up - threshold;

        if (scope > scope_max)
        {
          scope = scope_max;
        }

        scan_slices[i].points[j].intensity = scope / scope_max * MAX_PIXEL;
        scan_discontinuities_pcl.push_back(scan_slices[i].points[j]);
      }
    }
  }*/

  pcl::toPCLPointCloud2(scan_discontinuities_pcl, scan_pcl2);
  pcl_conversions::fromPCL(scan_pcl2, scan_discontinuities);

  return;
}

void OnlineCalibrationAlgorithm::acumAndProjectPoints(cv::Mat last_image, sensor_msgs::PointCloud2 scan_disc,
                                                      image_geometry::PinholeCameraModel cam_model,
                                                      std::string frame_lidar, std::string frame_odom,
                                                      ros::Time acquisition_time, tf::TransformListener& tf_listener,
                                                      cv::Mat& image_sobel_plot, cv::Mat& image_discontinuities)
{

  /****** variable declarations ******/
  int i, j, k;
  int rows = last_image.rows;
  int cols = last_image.cols;
  cv::Mat new_image(rows, cols, CV_8UC3, EMPTY_PIXEL);
  new_image.copyTo(image_discontinuities);
  float factor_color = 30.0; //TODO: get from parameter (only for plot)
  float sigma = 2000; //TODO: get from parameter
  float groung_filter_value = 1.2; //TODO: get from parameter
  int num_scans_acum = 150; //TODO: get from parameter
  //new_image.copyTo(image_sobel_plot);

  /********* transformation and acumulation of scans (including pcl conversions) *********/
  sensor_msgs::PointCloud2 scan_transformed;
  sensor_msgs::PointCloud2 cloud_odom;
  sensor_msgs::PointCloud2 cloud_camera;
  pcl::PCLPointCloud2 scan_pcl2;
  pcl::PCLPointCloud2 cloud_pcl2;
  static std::vector<pcl::PointCloud<pcl::PointXYZI> > scans_acum_pcl;
  static pcl::PointCloud<pcl::PointXYZI> scan_pcl_transformed;
  static pcl::PointCloud<pcl::PointXYZI> cloud_pcl;
  static pcl::PointCloud<pcl::PointXYZI> cloud_pcl_odom;
  static int count = 0;
  count++;
  try
  {
    ros::Duration duration(5.0);
    tf_listener.waitForTransform(frame_odom, frame_lidar, ros::Time::now(), duration);
    pcl_ros::transformPointCloud(frame_odom, scan_disc, scan_transformed, tf_listener);

    pcl_conversions::toPCL(scan_transformed, scan_pcl2);
    pcl::fromPCLPointCloud2(scan_pcl2, scan_pcl_transformed);

    if (count > num_scans_acum)
    {
      scans_acum_pcl.erase(scans_acum_pcl.begin());
    }

    cloud_pcl_odom.clear();
    scans_acum_pcl.push_back(scan_pcl_transformed);
    for (i = 0; i < scans_acum_pcl.size(); i++)
    {
      cloud_pcl_odom += scans_acum_pcl[i];
    }

    pcl::toPCLPointCloud2(cloud_pcl_odom, cloud_pcl2);
    pcl_conversions::fromPCL(cloud_pcl2, cloud_odom);
    cloud_odom.header.frame_id = frame_odom;

    tf_listener.waitForTransform(cam_model.tfFrame(), frame_odom, ros::Time(0), duration);
    pcl_ros::transformPointCloud(cam_model.tfFrame(), cloud_odom, cloud_camera, tf_listener);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }

  pcl_conversions::toPCL(cloud_camera, cloud_pcl2);
  pcl::fromPCLPointCloud2(cloud_pcl2, cloud_pcl);

  /********* project the cloud discontinuities in images *********/
  for (size_t i = 0; i < cloud_pcl.points.size(); ++i)
  {
    if (cloud_pcl.points[i].z > 0.0 && cloud_pcl.points[i].y < groung_filter_value)
    {
      // project into image plane
      cv::Point3d pt_cv(cloud_pcl.points[i].x, cloud_pcl.points[i].y, cloud_pcl.points[i].z);
      cv::Point2d uv;
      uv = cam_model.project3dToPixel(pt_cv);

      if (uv.x >= 0 && uv.y >= 0 && uv.x < cols && uv.y < rows)
      {
        // plot points in image
        static const int RADIUS = 2;
        float g = colorMapExp(cloud_pcl.points[i].z, sigma, MAX_PIXEL); //cloud_pcl.points[i].intensity;
        float r = cloud_pcl.points[i].intensity;
        float b = 0.0;
        image_discontinuities.at < cv::Vec3b > (uv.y, uv.x)[0] = r;
        image_discontinuities.at < cv::Vec3b > (uv.y, uv.x)[1] = g;
        image_discontinuities.at < cv::Vec3b > (uv.y, uv.x)[2] = b;
        r = EMPTY_PIXEL;
        g = cloud_pcl.points[i].intensity;
        b = EMPTY_PIXEL;
        cv::circle(image_sobel_plot, uv, RADIUS, CV_RGB(b, g, r), -1);
      }
    }
  }

  //cv::applyColorMap(image_discontinuities, image_discontinuities, cv::COLORMAP_JET);

  return;
}

void OnlineCalibrationAlgorithm::getDensityMaps(cv::Mat image, cv::Mat& density_map, cv::Rect roi)
{
  /****** variable declarations ******/
  /*int i, j, u, v;
   int rows = image.rows;
   int cols = image.cols;
   float intensity_total;
   float intensity_ratio;
   cv::Scalar rgb;
   cv::Mat roi_image;
   cv::Mat new_image(rows, cols, CV_8UC3, EMPTY_PIXEL);
   new_image.copyTo(density_map);

   for (i = 0; i < cols - roi.width; i += 2)
   {
   for (j = 0; j < rows - roi.height; j += 2)
   {
   roi.x = i;
   roi.y = j;
   roi_image = image(roi);
   rgb = cv::sum(roi_image);
   intensity_total = 0.3 * rgb[0] + 0.59 * rgb[1] + 0.11 * rgb[2];
   intensity_ratio = intensity_total / (roi.width * roi.height);
   v = j + roi.height / 2;
   u = i + roi.width / 2;
   density_map.at < cv::Vec3b > (v, u)[0] = intensity_ratio;
   density_map.at < cv::Vec3b > (v, u)[1] = intensity_ratio;
   density_map.at < cv::Vec3b > (v, u)[2] = intensity_ratio;
   }
   }
   cv::normalize(density_map, density_map, EMPTY_PIXEL, MAX_PIXEL, cv::NORM_MINMAX, -1);*/
  //cv::applyColorMap(density_map, density_map, cv::COLORMAP_JET);
  //output = A.mul(B); //element-wise multiplication
  return;
}

void OnlineCalibrationAlgorithm::getLocalMaximums(cv::Mat density_map, cv::Rect& roi_max)
{

  /*cv::Point uv_min, uv_max;
   double min, max;
   cv::cvtColor(density_map, density_map, CV_BGR2GRAY);
   cv::minMaxLoc(density_map, &min, &max, &uv_min, &uv_max);

   roi_max.x = uv_max.x;
   roi_max.y = uv_max.y;*/

  return;
}

void OnlineCalibrationAlgorithm::maskMatchingMutualInfo(cv::Mat& image_src, cv::Mat& image_dst, cv::Rect roi_max,
                                                        cv::Mat& correlation_map)
{

  // match templates with closs correlations
  /*cv::Mat image_dst_gray;
   cv::Mat image_src_gray;
   cv::cvtColor(image_src, image_src_gray, CV_BGR2GRAY);
   cv::cvtColor(image_dst, image_dst_gray, CV_BGR2GRAY);
   int result_rows = image_dst_gray.rows - roi_max.height + 1;
   int result_cols = image_dst_gray.cols - roi_max.width + 1;
   cv::Mat result(result_rows, result_cols, CV_32FC1, EMPTY_PIXEL);
   cv::Mat template_roi = image_src_gray(roi_max);
   cv::matchTemplate(image_dst_gray, template_roi, result, CV_TM_CCORR_NORMED);
   cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());*/
  //cv::normalize(result, result, EMPTY_PIXEL, MAX_PIXEL, cv::NORM_MINMAX, -1);
  //result.copyTo(correlation_map);
  // plot match roi in image
  /*double minVal;
   double maxVal;
   cv::Point minLoc;
   cv::Point maxLoc;
   cv::Rect roi_match;
   cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
   roi_match.x = maxLoc.x;
   roi_match.y = maxLoc.y;
   roi_match.width = roi_max.width;
   roi_match.height = roi_max.height;
   cv::rectangle(image_dst, roi_match, CV_RGB(0, 0, 255));*/

  // plot max density roi in image
  /*float r = EMPTY_PIXEL;
   float g = MAX_PIXEL;
   float b = EMPTY_PIXEL;
   cv::rectangle(image_src, roi_max, CV_RGB(b, g, r));*/

  return;
}

void OnlineCalibrationAlgorithm::featureMatching(cv::Mat& depth_map, cv::Mat& color_map, cv::Mat& image_matches)
{

  /********** match templates to get errors between images **********/
  /*cv::Mat result;
   int rows_templates = depth_map.rows / 2;
   int cols_templates = depth_map.cols / 5;
   int result_rows = depth_map.rows - rows_templates + 1;
   int result_cols = depth_map.cols - cols_templates + 1;
   result.create(result_rows, result_cols, CV_32FC1);

   // template 1
   int row_t = rows_templates / 2, col_t = cols_templates / 2;
   cv::Rect roi(col_t, row_t, col_t + cols_templates, row_t + rows_templates);
   cv::Mat template_1 = depth_map(roi);

   cv::matchTemplate(color_map, template_1, result, CV_TM_CCORR);
   cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
   result.copyTo(image_matches);*/
  /******************************************************************/

  /********** extract orb features **********/
  /*std::vector<cv::KeyPoint> kp_depth, kp_color;
   cv::Mat des_depth, des_color;

   // Detect ORB features and compute descriptors.
   cv::Ptr < cv::Feature2D > orb = cv::ORB::create(MAX_FEATURES);
   orb->detectAndCompute(depth_map, cv::Mat(), kp_depth, des_depth);
   orb->detectAndCompute(color_map, cv::Mat(), kp_color, des_color);

   // Match features.
   std::vector < cv::DMatch > matches;
   cv::Ptr < cv::DescriptorMatcher > matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
   matcher->match(des_depth, des_color, matches, cv::Mat());

   // Remove not so good matches
   const int num_good_matches = GOOD_MATCH_NUM;
   if (matches.size() > num_good_matches)
   matches.erase(matches.begin() + num_good_matches, matches.end());

   // Draw top matches
   cv::drawMatches(depth_map, kp_depth, color_map, kp_color, matches, image_matches);*/
  /******************************************/

//debug
//ROS_INFO("matches size: %d", matches.size());
//ROS_INFO("keypoints depth size: %d", kp_depth.size());
//ROS_INFO("keypoints color size: %d", kp_color.size());
  return;
}

////////////////////////////////////////////////////////////////////////////
//////////// Auxiliar functions

void cartesian2SphericalInDegrees(float x, float y, float z, float& range, float& azimuth, float& elevation)
{
  range = sqrt((x * x) + (y * y) + (z * z));

  azimuth = atan2(y, x) * 180.0 / M_PI;
  elevation = atan2(sqrt((x * x) + (y * y)), z) * 180.0 / M_PI;

  if (azimuth < 0)
    azimuth += 360.0;
  if (azimuth >= 360)
    azimuth -= 360;

  if (elevation < 0)
    elevation += 360.0;
  if (elevation >= 360)
    elevation -= 360;

  return;
}

void point2SphericalGrid(pcl::PointXYZI point, struct SensorConfiguration lidar_configuration, int& row, int& col)
{
  float range = 0.0;
  float elevation = 0.0;
  float azimuth = 0.0;

  cartesian2SphericalInDegrees(point.x, point.y, point.z, range, azimuth, elevation);

  if (azimuth > 180.0)
  {
    azimuth = azimuth - 360.0;
  }
  if (azimuth <= lidar_configuration.max_azimuth_angle && azimuth >= lidar_configuration.min_azimuth_angle
      && elevation <= lidar_configuration.max_elevation_angle && elevation >= lidar_configuration.min_elevation_angle)
  {
    col = (int)round(
        (azimuth - lidar_configuration.min_azimuth_angle) / lidar_configuration.grid_azimuth_angular_resolution);
    row = (int)round(
        (elevation - lidar_configuration.min_elevation_angle) / lidar_configuration.grid_elevation_angular_resolution);

    if (col < 0 || col >= lidar_configuration.num_of_azimuth_cells || row < 0
        || row >= lidar_configuration.num_of_elevation_cells)
    {
      row = col = INVALID_VALUE;
    }
  }
}

float colorMap(float dist, float factor, int base)
{
  float color_map;

  color_map = base - (dist / factor) * base;

  if (color_map < 0.0)
  {
    color_map = 0.0;
  }

  return color_map;
}

float colorMapExp(float dist, float sigma, int base)
{
  float color_map;
  float ini = 0.0;

  dist = dist * 100.0 + 200.0; //conversion to cm and displacement

  float normalization_factor =  float(base) / ((1 / (sqrt(2*M_PI) * sigma)) * exp(-(pow(ini, 2) / (2 * pow(sigma, 2)))));
  float exp_value =  (1 / (sqrt(2*M_PI) * sigma)) * exp(-(pow(dist, 2) / (2 * pow(sigma, 2))));

  color_map = exp_value * normalization_factor;

  return color_map;
}


#include "online_calibration_alg.h"

float colorMap(float dist, float factor, int base);

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
void OnlineCalibrationAlgorithm::sensorFusion(cv::Mat last_image, sensor_msgs::PointCloud2 msg,
                                              image_geometry::PinholeCameraModel cam_model, std::string frame_id,
                                              ros::Time acquisition_time, tf::TransformListener& tf_listener,
                                              cv::Mat& plot_image)
{

  /****** variable declarations ******/
  int i, j, k;
  float factor_color = 4.0; //TODO: get from parameter
  int rows = last_image.rows;
  int cols = last_image.cols;
  /*int index[rows][cols];
  for (i = 0; i < cols; i++)
    for (j = 0; j < rows; j++)
      index[j][i] = NO_INDEX;*/

  /****** get transformation, and transform point cloud ******/
  sensor_msgs::PointCloud2 transform_pc;
  try
  {
    ros::Duration timeout(1.0 / 30);
    tf_listener.waitForTransform(cam_model.tfFrame(), frame_id, acquisition_time, timeout);
    pcl_ros::transformPointCloud(cam_model.tfFrame(), msg, transform_pc, tf_listener);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }

  /****** generate new point clouds with PCL format ******/
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcl_orig(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PCLPointCloud2 aux_input;
  pcl_conversions::toPCL(transform_pc, aux_input);
  pcl::fromPCLPointCloud2(aux_input, *cloud_pcl);
  pcl_conversions::toPCL(msg, aux_input);
  pcl::fromPCLPointCloud2(aux_input, *cloud_pcl_orig);

  /****** get field of view and save index correspondence between cloud and image ******/
  for (size_t i = 0; i < cloud_pcl->points.size(); ++i)
  {
    if (cloud_pcl->points[i].z > 0.0)
    {
      // project into image plane
      cv::Point3d pt_cv(cloud_pcl->points[i].x, cloud_pcl->points[i].y, cloud_pcl->points[i].z);
      cv::Point2d uv;
      uv = cam_model.project3dToPixel(pt_cv);

      if (uv.x >= 0 && uv.y >= 0 && uv.x < cols && uv.y < rows)
      {
        //// Save (x, y, z) and (u, v) information
        //index[(int)uv.y][(int)uv.x] = i;

        // plot points in image
        static const int RADIUS = 3;
        float r = 0.0;
        float g = colorMap(cloud_pcl->points[i].z, factor_color, 255);
        float b = 0.0;
        cv::circle(plot_image, uv, RADIUS, CV_RGB(r, g, b), -1);
      }
    }
  }

  return;
}

void OnlineCalibrationAlgorithm::featureMatching(cv::Mat& depth_map, cv::Mat& color_map, cv::Mat& image_matches)
{

  /********** sobel filter **********/
  /*cv::Mat depth_map_gray;
   cv::Mat color_map_gray;
   cv::Mat grad;
   cv::Mat grad_x, grad_y;
   cv::Mat abs_grad_x, abs_grad_y;
   int scale = 1;
   int delta = 0;
   int ddepth = CV_16S;

   cv::GaussianBlur(depth_map, depth_map, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
   cv::cvtColor(depth_map, depth_map_gray, CV_BGR2GRAY);
   cv::Sobel(depth_map_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
   cv::Sobel(depth_map_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
   convertScaleAbs(grad_x, abs_grad_x);
   convertScaleAbs(grad_y, abs_grad_y);
   addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
   grad.copyTo(depth_map);

   cv::GaussianBlur(color_map, color_map, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
   cv::cvtColor(color_map, color_map_gray, CV_BGR2GRAY);
   cv::Sobel(color_map_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
   cv::Sobel(color_map_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
   convertScaleAbs(grad_x, abs_grad_x);
   convertScaleAbs(grad_y, abs_grad_y);
   addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
   grad.copyTo(color_map);*/
  /**********************************/

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

float colorMap(float dist, float factor, int base)
{
  float color_map;

  color_map = (dist / factor) * base;

  if (color_map > base)
  {
    color_map = base;
  }

  return color_map;
}

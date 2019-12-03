#include "online_calibration_alg_node.h"

OnlineCalibrationAlgNode::OnlineCalibrationAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<OnlineCalibrationAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 20; //in [Hz]
  this->frame_lidar_ = "/velodyne"; //TODO: get from param
  this->frame_map_ = "/map"; //TODO: get from param
  cvInitFont(&this->font_, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0);
  image_transport::ImageTransport it_(this->public_node_handle_);

  this->save_data_ = false;
  this->out_path_image_ =
      "/home/mice85/aurova-lab/aurova_ws/src/aurova_preprocessed/online_calibration/scripts/images/input/raw_data_07/image";
  this->out_path_scan_ =
      "/home/mice85/aurova-lab/aurova_ws/src/aurova_preprocessed/online_calibration/scripts/images/input/raw_data_07/scan";
  this->out_path_tf_ =
      "/home/mice85/aurova-lab/aurova_ws/src/aurova_preprocessed/online_calibration/scripts/images/input/raw_data_07/tf";
  // [init publishers]
  this->plot_publisher_ = it_.advertise("/plot_out", 1);
  //this->sobel_publisher_ = it_.advertise("/sobel_out", 1);
  //this->discnt_publisher_ = it_.advertise("/discnt_out", 1);
  //this->soplt_publisher_ = it_.advertise("/sobel_plt_out", 1);

  // [init subscribers]
  this->camera_subscriber_ = it_.subscribeCamera("/image", 1, &OnlineCalibrationAlgNode::cb_imageInfo, this);
  this->lidar_subscriber_ = this->public_node_handle_.subscribe("/velodyne_points", 1,
                                                                &OnlineCalibrationAlgNode::cb_lidarInfo, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

OnlineCalibrationAlgNode::~OnlineCalibrationAlgNode(void)
{
  // [free dynamic memory]
}

void OnlineCalibrationAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]

}

/*  [subscriber callbacks] */
void OnlineCalibrationAlgNode::cb_imageInfo(const sensor_msgs::ImageConstPtr& image_msg,
                                            const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  this->alg_.lock();

  //////////////////////////////////////////////////////
  //// transform image from ROS to OpenCV and save in class variables
  try
  {

    this->input_bridge_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    this->input_bridge_plt_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    this->last_image_ = this->input_bridge_->image;
    this->plot_image_ = this->input_bridge_plt_->image;
  }
  catch (cv_bridge::Exception& ex)
  {
    ROS_ERROR("[draw_frames] Failed to convert image");
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
    return;
  }

  //////////////////////////////////////////////////////
  //// save camera model and timestamp in class variables.
  this->cam_model_.fromCameraInfo(info_msg);
  this->acquisition_time_ = info_msg->header.stamp;

  this->alg_.unlock();
}

void OnlineCalibrationAlgNode::cb_lidarInfo(const sensor_msgs::PointCloud2::ConstPtr& scan)
{
  this->alg_.lock();

  double ini, end;
  ini = ros::Time::now().toSec();

  //////////////////////////////////////////////////////////////////
  //// preprocess all the data (image and lidar-scan)
  /*cv::Mat image_sobel;
   cv::Mat image_sobel_plot;
   cv::Mat image_discontinuities;
   sensor_msgs::PointCloud2 scan_discontinuities;
   this->alg_.filterSensorsData(this->last_image_, *scan, this->cam_model_, this->frame_lidar_, this->acquisition_time_,
   this->tf_listener_, scan_discontinuities, this->plot_image_, image_sobel,
   image_sobel_plot);

   scan_discontinuities.header = scan->header;
   this->alg_.acumAndProjectPoints(this->last_image_, scan_discontinuities, this->cam_model_, this->frame_lidar_,
   this->frame_odom_, this->acquisition_time_, this->tf_listener_, image_sobel_plot,
   image_discontinuities);*/

  //////////////////////////////////////////////////////////////////
  //// get match features (and errors) between sobel and disc. info
  /*int mask_width = 48; // TODO: get from parameter (always pair)
   int mask_height = 48;
   cv::Mat density_map;
   cv::Mat correlation_map;
   cv::Rect roi;
   cv::Rect roi_max;
   roi_max.width = roi.width = mask_width;
   roi_max.height = roi.height = mask_height;
   this->alg_.getDensityMaps(image_discontinuities, density_map, roi);
   this->alg_.getLocalMaximums(density_map, roi_max);
   this->alg_.maskMatchingMutualInfo(image_discontinuities, image_sobel, roi_max, correlation_map);*/

  //////////////////////////////////////////////////////////////////
  //// apply control law of VS
  //////////////////////////////////////////////////////////////////
  //// integrate velocities to modify [t|R]
  //////////////////////////////////////////////////////////////////
  //// publish in image topics
  //std_msgs::Header header; // empty header
  //header.stamp = ros::Time::now(); // time
  //cv_bridge::CvImage output_bridge;
  //output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_discontinuities);
  //this->discnt_publisher_.publish(output_bridge.toImageMsg());
  //output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_sobel);
  //this->sobel_publisher_.publish(output_bridge.toImageMsg());
  //output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_sobel_plot);
  //this->soplt_publisher_.publish(output_bridge.toImageMsg());
  this->plot_publisher_.publish(this->input_bridge_->toImageMsg());

  //////////////////////////////////////////////////////////
  // get transform info
  tf::StampedTransform transform_map;
  tf::StampedTransform transform_cam;
  try
  {
    ros::Duration duration(5.0);
    this->tf_listener_.lookupTransform(this->frame_map_, this->frame_lidar_, ros::Time(0), transform_map);
    this->tf_listener_.lookupTransform(this->cam_model_.tfFrame(), this->frame_map_, ros::Time(0), transform_cam);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }

  float transform_map_x = transform_map.getOrigin().x();
  float transform_map_y = transform_map.getOrigin().y();
  float transform_map_z = transform_map.getOrigin().z();
  double transform_map_r = 0.0;
  double transform_map_p = 0.0;
  double transform_map_w = 0.0;
  tf::Quaternion quaternion_aux = transform_map.getRotation();
  tf::Matrix3x3 matrix(quaternion_aux);
  matrix.getRPY(transform_map_r, transform_map_p, transform_map_w);

  float transform_cam_x = transform_cam.getOrigin().x();
  float transform_cam_y = transform_cam.getOrigin().y();
  float transform_cam_z = transform_cam.getOrigin().z();
  double transform_cam_r = 0.0;
  double transform_cam_p = 0.0;
  double transform_cam_w = 0.0;
  tf::Quaternion quaternion_aux2 = transform_cam.getRotation();
  tf::Matrix3x3 matrix2(quaternion_aux2);
  matrix2.getRPY(transform_cam_r, transform_cam_p, transform_cam_w);
  //////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////
  // save images, scan, and transform
  pcl::PCLPointCloud2 scan_pcl2;
  static pcl::PointCloud<pcl::PointXYZI> scan_pcl;

  pcl_conversions::toPCL(*scan, scan_pcl2);
  pcl::fromPCLPointCloud2(scan_pcl2, scan_pcl);

  if (this->save_data_)
  {
    static int cont = 0;

    std::ostringstream out_path_image;
    std::ostringstream out_path_scan;
    std::ostringstream out_path_tf;
    std::ostringstream out_tf;
    std::ofstream file_tf;


    out_path_image << this->out_path_image_ << cont << ".jpg";
    out_path_scan << this->out_path_scan_ << cont << ".pcd";
    out_path_tf << this->out_path_tf_ << cont << ".csv";

    cv::imwrite(out_path_image.str(), this->last_image_);
    pcl::io::savePCDFileASCII(out_path_scan.str(), scan_pcl);

    file_tf.open(out_path_tf.str().c_str(), std::ofstream::trunc);
    out_tf << transform_map_x << ", " << transform_map_y << ", " << transform_map_z << ", " << transform_map_r << ", "
        << transform_map_p << ", " << transform_map_w << "\n";
    out_tf << transform_cam_x << ", " << transform_cam_y << ", " << transform_cam_z << ", " << transform_cam_r << ", "
        << transform_cam_p << ", " << transform_cam_w << "\n";
    file_tf << out_tf.str();
    file_tf.close();

    cont++;
  }

  // loop time
  end = ros::Time::now().toSec();
  ROS_INFO("duration: %f", end - ini);

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void OnlineCalibrationAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void OnlineCalibrationAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < OnlineCalibrationAlgNode > (argc, argv, "online_calibration_alg_node");
}

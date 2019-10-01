#include "online_calibration_alg_node.h"

OnlineCalibrationAlgNode::OnlineCalibrationAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<OnlineCalibrationAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 20; //in [Hz]
  this->frame_lidar_ = "/velodyne"; //TODO: get from param
  this->frame_lidar_calib_ = "/velodyne_calib"; //TODO: get from param
  this->frame_odom_ = "/odom"; //TODO: get from param
  cvInitFont(&this->font_, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0);
  image_transport::ImageTransport it_(this->public_node_handle_);

  // [init publishers]
  this->plot_publisher_ = it_.advertise("/plot_out", 1);
  this->edges_publisher_ = it_.advertise("/edges_out", 1);
  this->sobel_publisher_ = it_.advertise("/sobel_out", 1);
  this->soplt_publisher_ = it_.advertise("/sobel_plt_out", 1);

  // [init subscribers]
  this->camera_subscriber_ = it_.subscribeCamera("/image", 1, &OnlineCalibrationAlgNode::cb_imageInfo, this);
  this->lidar_subscriber_ = this->public_node_handle_.subscribe("/velodyne_points", 1,
                                                                &OnlineCalibrationAlgNode::cb_lidarInfo, this);

  this->alg_.twist_change_calib_.x = 0.0;
  this->alg_.twist_change_calib_.y = 0.0;
  this->alg_.twist_change_calib_.z = 0.0;
  this->alg_.twist_change_calib_.r = 0.0;
  this->alg_.twist_change_calib_.p = 0.0;
  this->alg_.twist_change_calib_.w = 0.0;
  this->alg_.twist_change_calib_.delta_t = 0.0;

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
  //this->cb_sendTransform(this->alg_.twist_change_calib_);

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

  //////////////////////////////////////////////////////////////////
  //// preprocess all the data (image and lidar-scan)
  cv::Mat image_sobel;
  cv::Mat image_sobel_plot;
  cv::Mat image_discontinuities;
  sensor_msgs::PointCloud2 scan_discontinuities;
  this->alg_.filterSensorsData(this->last_image_, *scan, this->cam_model_, this->frame_lidar_,
                               this->acquisition_time_, this->tf_listener_, scan_discontinuities, this->plot_image_,
                               image_sobel, image_sobel_plot);

  scan_discontinuities.header = scan->header;
  this->alg_.acumAndProjectPoints(this->last_image_, scan_discontinuities, this->cam_model_, this->frame_lidar_,
                                  this->frame_odom_, this->acquisition_time_, this->tf_listener_, image_sobel_plot,
                                  image_discontinuities);

  //////////////////////////////////////////////////////////////////
  //// get match features (and errors) between sobel and disc. info
  //////////////////////////////////////////////////////////////////
  //// apply control law of VS
  //////////////////////////////////////////////////////////////////
  //// integrate velocities to modify [t|R]
  //int key = this->alg_.getch();
  //ROS_INFO("character %c", key);
  //this->alg_.mapKeysToVelocities(key, this->alg_.twist_change_calib_);
  //this->cb_sendTransform(this->alg_.twist_change_calib_);

  //////////////////////////////////////////////////////////////////
  //// publish in image topics
  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  cv_bridge::CvImage output_bridge;
  output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_discontinuities);
  this->edges_publisher_.publish(output_bridge.toImageMsg());
  output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, image_sobel);
  this->sobel_publisher_.publish(output_bridge.toImageMsg());
  output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_sobel_plot);
  this->soplt_publisher_.publish(output_bridge.toImageMsg());
  this->plot_publisher_.publish(this->input_bridge_plt_->toImageMsg());

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */
void OnlineCalibrationAlgNode::cb_sendTransform(struct Twist twist_change_calib)
{
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = this->frame_lidar_;
  transform.child_frame_id = this->frame_lidar_calib_;
  transform.header.stamp = ros::Time::now();
  transform.transform.translation.x = twist_change_calib.x * twist_change_calib.delta_t;
  transform.transform.translation.y = twist_change_calib.y * twist_change_calib.delta_t;
  transform.transform.translation.z = twist_change_calib.z * twist_change_calib.delta_t;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(twist_change_calib.r * twist_change_calib.delta_t,
                                                          twist_change_calib.p * twist_change_calib.delta_t,
                                                          twist_change_calib.w * twist_change_calib.delta_t);
  transform.transform.rotation.x = quaternion[0];
  transform.transform.rotation.y = quaternion[1];
  transform.transform.rotation.z = quaternion[2];
  transform.transform.rotation.w = quaternion[3];

  this->tf_broadcaster_.sendTransform(transform);
}

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

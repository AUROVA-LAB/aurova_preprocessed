#include "online_calibration_alg_node.h"

OnlineCalibrationAlgNode::OnlineCalibrationAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<OnlineCalibrationAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]
  this->frame_lidar_ = "/velodyne"; //TODO: get from param
  this->frame_odom_ = "/odom"; //TODO: get from param
  cvInitFont(&this->font_, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0);
  image_transport::ImageTransport it_(this->public_node_handle_);

  // [init publishers]
  this->plot_publisher_ = it_.advertise("/plot_out", 1);
  this->edges_publisher_ = it_.advertise("/edges_out", 1);
  this->sobel_publisher_ = it_.advertise("/sobel_out", 1);

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

void OnlineCalibrationAlgNode::cb_lidarInfo(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  this->alg_.lock();

  //////////////////////////////////////////////////////////////////
  //// preprocessed all the data
  cv::Mat depth_map;
  cv::Mat index_to_cloud;
  cv::Mat image_sobel;
  cv::Mat image_discontinuities;
  sensor_msgs::PointCloud2 scan_discontinuities;
  this->alg_.depthImageFromLidar(this->last_image_, *msg, this->cam_model_, this->frame_lidar_, this->acquisition_time_,
                                 this->tf_listener_, index_to_cloud, depth_map);
  this->alg_.preprocessCloudAndImage(this->last_image_, *msg, depth_map, index_to_cloud, image_sobel,
                                     image_discontinuities, scan_discontinuities);

  //////////////////////////////////////////////////////////////////
  //// representation of camera and lidar information (TODO: put into previous function!!)
  this->alg_.sensorFusion(this->last_image_, *msg, this->cam_model_, this->frame_lidar_, this->frame_odom_,
                          this->acquisition_time_, this->tf_listener_, this->plot_image_);

  //////////////////////////////////////////////////////////////////
  //// get match features (and errors) between laser and lidar info

  //////////////////////////////////////////////////////////////////
  //// apply control law of VS for modify [t|R]

  //////////////////////////////////////////////////////////////////
  //// publish in image topics
  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  //float resize_factor = 8.0;
  //cv::Mat image_discontinuities_plt;
  //cv::resize(image_discontinuities, image_discontinuities_plt, cv::Size(), 1.0, resize_factor);
  cv_bridge::CvImage output_bridge;
  output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::/*MONO8*/RGB8, image_discontinuities);
  this->edges_publisher_.publish(output_bridge.toImageMsg());
  output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, image_sobel);
  this->sobel_publisher_.publish(output_bridge.toImageMsg());
  this->plot_publisher_.publish(this->input_bridge_plt_->toImageMsg());

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

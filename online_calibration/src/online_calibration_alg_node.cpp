#include "online_calibration_alg_node.h"

OnlineCalibrationAlgNode::OnlineCalibrationAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<OnlineCalibrationAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]
  this->frame_id_ = "/velodyne"; //TODO: get from param
  cvInitFont(&this->font_, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0);
  image_transport::ImageTransport it_(this->public_node_handle_);

  // [init publishers]
  this->plot_publisher_ = it_.advertise("/plot_out", 1);
  this->depth_publisher_ = it_.advertise("/depth_out", 1);
  this->color_publisher_ = it_.advertise("/color_out", 1);
  this->matches_publisher_ = it_.advertise("/matches_out", 1);

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

  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  cv::Mat depth_map_plot;
  cv::Mat color_map_plot;
  cv::Mat image_matches_plot;
  cv::Mat depth_map;
  cv::Mat color_map;
  cv::Mat image_matches;

  //////////////////////////////////////////////////////
  //// fusion camera and lidar information
  this->alg_.sensorFusion(this->last_image_, *msg, this->cam_model_, this->frame_id_, this->acquisition_time_,
                          this->tf_listener_, depth_map, color_map, this->plot_image_);

  //////////////////////////////////////////////////////
  //// get match features between laser and lidar info
  float resize_factor = 6.0; // TODO: get from parameter
  cv::resize(depth_map, depth_map_plot, cv::Size(), 1.0, resize_factor);
  cv::resize(color_map, color_map_plot, cv::Size(), 1.0, resize_factor);
  this->alg_.featureMatching(depth_map_plot, color_map_plot, image_matches);

  //////////////////////////////////////////////////////
  //// publish in image topics
  cv_bridge::CvImage output_bridge;
  cv::resize(image_matches, image_matches_plot, cv::Size(), 1.0, 1.0);
  this->plot_publisher_.publish(this->input_bridge_plt_->toImageMsg());
  output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, depth_map_plot);
  this->depth_publisher_.publish(output_bridge.toImageMsg());
  output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, color_map_plot);
  this->color_publisher_.publish(output_bridge.toImageMsg());
  output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_matches_plot);
  this->matches_publisher_.publish(output_bridge.toImageMsg());

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

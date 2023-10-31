#include "kitti_preprocess_alg_node.h"

KittiPreprocessAlgNode::KittiPreprocessAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<KittiPreprocessAlgorithm>()
{
  //init class attributes if necessary
  this->public_node_handle_.getParam("/geo_localization/lat_zero", this->lat_zero_);
  this->public_node_handle_.getParam("/geo_localization/lon_zero", this->lon_zero_);

  if(!this->private_node_handle_.getParam("rate", this->config_.rate))
  {
    ROS_WARN("KittiPreprocessAlgNode::KittiPreprocessAlgNode: param 'rate' not found");
  }
  else
    this->setRate(this->config_.rate);

  // [init publishers]
  this->img_range_publisher_ = this->private_node_handle_.advertise<sensor_msgs::Image>("ouster/range_image", 1);
  this->img_reflec_publisher_ = this->private_node_handle_.advertise<sensor_msgs::Image>("ouster/reflec_image", 1);
  this->img_nearir_publisher_ = this->private_node_handle_.advertise<sensor_msgs::Image>("ouster/nearir_image", 1);
  this->img_signal_publisher_ = this->private_node_handle_.advertise<sensor_msgs::Image>("ouster/signal_image", 1);
  this->img_x_publisher_ = this->private_node_handle_.advertise<sensor_msgs::Image>("ouster/x_image", 1);
  this->img_y_publisher_ = this->private_node_handle_.advertise<sensor_msgs::Image>("ouster/y_image", 1);
  this->img_z_publisher_ = this->private_node_handle_.advertise<sensor_msgs::Image>("ouster/z_image", 1);
  this->odometry_gps_publisher_ = this->private_node_handle_.advertise<nav_msgs::Odometry>("odometry_gps", 1);
  this->odom_publisher_ = this->private_node_handle_.advertise<nav_msgs::Odometry>("odom", 1);
  
  // [init subscribers]
  this->fix_subscriber_ = this->private_node_handle_.subscribe("/kitti/oxts/gps/fix", 1, &KittiPreprocessAlgNode::fix_callback, this);
  pthread_mutex_init(&this->fix_mutex_,NULL);

  //this->pointcloud_subscriber_ = this->private_node_handle_.subscribe("/kitti/velo/pointcloud", 1, &KittiPreprocessAlgNode::pointcloud_callback, this);
  //pthread_mutex_init(&this->pointcloud_mutex_,NULL);
  this->pc_sync_subscriber_.subscribe(this->private_node_handle_,"/kitti/velo/pointcloud",1);
  this->info_sync_subscriber_.subscribe(this->private_node_handle_,"/kitti/camera_gray_left/camera_info",1);
  this->img_sync_subscriber_.subscribe(this->private_node_handle_,"/kitti/camera_gray_left/image_raw",1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
  static message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sync_subscriber_, img_sync_subscriber_, info_sync_subscriber_);
  sync.registerCallback(boost::bind(&KittiPreprocessAlgNode::pointcloud_callback,this, _1, _2, _3));

  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

KittiPreprocessAlgNode::~KittiPreprocessAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->fix_mutex_);
  pthread_mutex_destroy(&this->pointcloud_mutex_);
}

void KittiPreprocessAlgNode::mainNodeThread(void)
{
  //lock access to algorithm if necessary
  this->alg_.lock();
  ROS_DEBUG("KittiPreprocessAlgNode::mainNodeThread");
  // [fill msg structures]
  // Initialize the topic message structure
  //this->odometry_gps_Odometry_msg_.data = my_var;

  // Initialize the topic message structure
  //this->odom_Odometry_msg_.data = my_var;
  
  //tf_listener example BEGIN
  try{
    std::string target_frame             = "base_link";
    std::string source_frame             = "odom";
    ros::Time time                       = ros::Time(0);
    ros::Duration timeout                = ros::Duration(0.1);
    ros::Duration polling_sleep_duration = ros::Duration(0.01);
    this->alg_.unlock();
    bool tf_exists = this->tf_listener_.waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration);
    this->alg_.lock();
    if(tf_exists){
      tf::StampedTransform tf_parent_child;
      this->tf_listener_.lookupTransform(source_frame, target_frame, time, tf_parent_child);

      static int seq = 0;
      this->odom_Odometry_msg_.header.seq = seq;
      this->odom_Odometry_msg_.header.stamp = ros::Time::now();
      this->odom_Odometry_msg_.header.frame_id = "odom";
      this->odom_Odometry_msg_.pose.pose.position.x = tf_parent_child.getOrigin().x();
      this->odom_Odometry_msg_.pose.pose.position.y = tf_parent_child.getOrigin().y();
      this->odom_Odometry_msg_.pose.pose.position.z = 0.0;//tf_parent_child.getOrigin().z();
      this->odom_Odometry_msg_.pose.pose.orientation.x = 0.0;//tf_parent_child.getRotation().x();
      this->odom_Odometry_msg_.pose.pose.orientation.y = 0.0;//tf_parent_child.getRotation().y();
      this->odom_Odometry_msg_.pose.pose.orientation.z = tf_parent_child.getRotation().z();
      this->odom_Odometry_msg_.pose.pose.orientation.w = tf_parent_child.getRotation().w();
      this->odom_publisher_.publish(this->odom_Odometry_msg_);
      seq++;

    }else{
      ROS_WARN("No transform found from '%s' to '%s'", source_frame.c_str(), target_frame.c_str()); }
  }catch (tf::TransformException &ex){
    ROS_ERROR("TF Exception: %s",ex.what()); }
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->odometry_gps_publisher_.publish(this->odometry_gps_Odometry_msg_);
  
  this->alg_.unlock();
}

/*  [subscriber callbacks] */
void KittiPreprocessAlgNode::fix_callback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg)
{
  //ROS_INFO("KittiPreprocessAlgNode::fix_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  this->fix_mutex_enter();

  Ellipsoid utm;
  double utm_x;
  double utm_y;
  double zero_x;
  double zero_y;
  char utm_zone[32];
  int ref_ellipsoid = 23;

  utm.LLtoUTM(ref_ellipsoid, fix_msg->latitude, fix_msg->longitude, utm_y, utm_x, utm_zone);
  utm.LLtoUTM(ref_ellipsoid, this->lat_zero_, this->lon_zero_, zero_y, zero_x, utm_zone);

  ///////////////////////////////////////////////////////////////////////////////
  //// FOR REPRESENTATION PURPO0SES
  Eigen::Vector3d map_orientations_RPY = Eigen::Vector3d::Zero();
  map_orientations_RPY(0) = 0.0;
  map_orientations_RPY(1) = -1.57;
  map_orientations_RPY(2) = 0.0;

  // Converting to quarternion to fill the ROS message
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(map_orientations_RPY(0), map_orientations_RPY(1),
                                                          map_orientations_RPY(2));

  static int seq = 0;
  this->odometry_gps_Odometry_msg_.header.stamp = ros::Time::now();
  this->odometry_gps_Odometry_msg_.header.seq = seq;
  this->odometry_gps_Odometry_msg_.header.frame_id = "map";
  this->odometry_gps_Odometry_msg_.pose.pose.position.x = utm_x - zero_x;
  this->odometry_gps_Odometry_msg_.pose.pose.position.y = utm_y - zero_y;
  this->odometry_gps_Odometry_msg_.pose.pose.position.z = 0.0;
  this->odometry_gps_Odometry_msg_.pose.pose.orientation.x = quaternion[0];
  this->odometry_gps_Odometry_msg_.pose.pose.orientation.y = quaternion[1];
  this->odometry_gps_Odometry_msg_.pose.pose.orientation.z = quaternion[2];
  this->odometry_gps_Odometry_msg_.pose.pose.orientation.w = quaternion[3];
  this->odometry_gps_Odometry_msg_.pose.covariance[0] = fix_msg->position_covariance[0];
  this->odometry_gps_Odometry_msg_.pose.covariance[7] = fix_msg->position_covariance[4];
  this->odometry_gps_Odometry_msg_.pose.covariance[14] = fix_msg->position_covariance[8];
  seq++;

  std::cout << "X: " << this->odometry_gps_Odometry_msg_.pose.pose.position.x << ", Y: " << this->odometry_gps_Odometry_msg_.pose.pose.position.y << std::endl;

  this->odometry_gps_publisher_.publish(this->odometry_gps_Odometry_msg_);

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  this->alg_.unlock();
  this->fix_mutex_exit();
}

void KittiPreprocessAlgNode::fix_mutex_enter(void)
{
  pthread_mutex_lock(&this->fix_mutex_);
}

void KittiPreprocessAlgNode::fix_mutex_exit(void)
{
  pthread_mutex_unlock(&this->fix_mutex_);
}

void KittiPreprocessAlgNode::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& scan, 
                                                 const sensor_msgs::Image::ConstPtr& img_msg, 
                                                 const sensor_msgs::CameraInfo::ConstPtr& info_msg)
{
  //ROS_INFO("KittiPreprocessAlgNode::pointcloud_callback: New Message Received");
  
  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  this->pointcloud_mutex_enter();

  //// CONFIGURATION VARIAVBLES
  float max_elevation_angle = 114.5;// -24.5 dg
  float min_elevation_angle = 88;// +2 dg
  float max_azimuth_angle = 213;
  float min_azimuth_angle = 147;
  float grid_azimuth_angular_resolution = 0.23;//0.2296;
  float grid_elevation_angular_resolution = 0.21; //0.42;
  int num_of_azimuth_cells = 2 + (max_azimuth_angle - min_azimuth_angle) / grid_azimuth_angular_resolution; // +2 instead of +1, to be divisible with 32.
  int num_of_elevation_cells = 2 + (max_elevation_angle - min_elevation_angle) / grid_elevation_angular_resolution;
  float min_range = 2.0;
  float max_range = 100.0;
  //float M_PI = 3.1415;

  //////////////////////////////////////////////////////////////////////////////
  //// PARSE TO OPEN CV AND CAMERA MOEL FORMAT FROM MSG
  cv_bridge::CvImagePtr cv_gray;
  try
  {
    cv_gray = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO16);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat img_gray  = cv_gray->image;
  this->cam_model_.fromCameraInfo(info_msg);
  //std::cout << "FRAME: " << this->cam_model_.tfFrame() << std::endl;

  //////////////////////////////////////////////////////////////////////////////
  //// TRANSFORM PC TO CAMERA FRAME
  sensor_msgs::PointCloud2 scan_transformed;
  try
  {
    ros::Duration duration(5.0);
    this->tf_listener_.waitForTransform(this->cam_model_.tfFrame(), "velo_link", ros::Time::now(), duration);
    pcl_ros::transformPointCloud(this->cam_model_.tfFrame(), *scan, scan_transformed, this->tf_listener_);

  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }

  //////////////////////////////////////////////////////////////////////////////
  //// PARSE TO PCL FORMAT, INCLUDING INTENSITY
  sensor_msgs::PointCloud2 scan_new;
  sensor_msgs::PointCloud2 scan_new2;
  scan_new = *scan;
  scan_new2 = scan_transformed;
  scan_new.fields[3].name = "intensity";  
  scan_new2.fields[3].name = "intensity";
  static pcl::PointCloud<pcl::PointXYZI> scan_pcl;
  static pcl::PointCloud<pcl::PointXYZI> scan_transformed_pcl;
  pcl::fromROSMsg(scan_new, scan_pcl);
  pcl::fromROSMsg(scan_new2, scan_transformed_pcl);

  //////////////////////////////////////////////////////////////////////////////
  //// CREATE OPEN CV MAT.
  int rows = num_of_elevation_cells;
  int cols = num_of_azimuth_cells;
  cv::Mat img_range = cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));
  cv::Mat img_reflec = cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));
  cv::Mat img_x = cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));
  cv::Mat img_y = cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));
  cv::Mat img_z = cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));
  cv::Mat img_range_bk;
  cv::Mat img_reflec_bk;

  //////////////////////////////////////////////////////////////////////////////
  //// FILL SPHERICAL OPEN CV IMAGE
  float range = 0.0;
  float elevation = 0.0;
  float azimuth = 0.0;
  int row, col;
  for (size_t i = 0; i < scan_pcl.points.size(); ++i)
  {
    pcl::PointXYZI point = scan_pcl.points.at(i);

    range = sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));

    azimuth = atan2(point.y, point.x) * 180.0 / M_PI;
    elevation = atan2(sqrt((point.x * point.x) + (point.y * point.y)), point.z) * 180.0 / M_PI;

    azimuth = azimuth - 180.0; // To correct projection
    if (azimuth < 0)
      azimuth += 360.0;
    if (azimuth >= 360)
      azimuth -= 360;

    if (elevation < 0)
      elevation += 360.0;
    if (elevation >= 360)
      elevation -= 360;

    //Filtering points of our own vehicle and out of desired FOV
    if (range >= min_range
        && range <= max_range
        && azimuth <= max_azimuth_angle
        && azimuth >= min_azimuth_angle
        && elevation <= max_elevation_angle
        && elevation >= min_elevation_angle)
    {
      col = (int) round((azimuth - min_azimuth_angle) / grid_azimuth_angular_resolution);
      row = (int) round((elevation - min_elevation_angle) / grid_elevation_angular_resolution);

      if (col >= 0 && col < num_of_azimuth_cells && row >= 0 && row < num_of_elevation_cells)
      {
        if (range > max_range) range = max_range;
        img_range.at<ushort>(row, col) = pow(2,16) * (range / max_range);

        //img_reflec.at<ushort>(row, col) = pow(2,16) * scan_pcl.points.at(i).intensity;

        img_x.at<ushort>(row, col) = pow(2,16) * ((scan_pcl.points.at(i).x + max_range) / (2 * max_range));
        img_y.at<ushort>(row, col) = pow(2,16) * ((scan_pcl.points.at(i).y + max_range) / (2 * max_range));
        img_z.at<ushort>(row, col) = pow(2,16) * ((scan_pcl.points.at(i).z + max_range) / (2 * max_range));

        cv::Point2d uv;
        cv::Point3d pt_cv(scan_transformed_pcl.points[i].x, scan_transformed_pcl.points[i].y,
                          scan_transformed_pcl.points[i].z);
        uv = this->cam_model_.project3dToPixel(pt_cv);
        if (uv.x >= 0 && uv.y >= 0 && uv.x < img_gray.cols && uv.y < img_gray.rows)
        {
          img_reflec.at<ushort>(row, col) = img_gray.at<ushort>(uv.y, uv.x);
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //// Filter black holes.
  int S = 3;
  for (int r = 0; r < 2; r++){
    img_range_bk = img_range.clone();
    img_reflec_bk = img_reflec.clone();
    for (int j = S; j < rows-S; j++){
      for (int i = S; i < cols-S; i++){
        if (img_range_bk.at<ushort>(j, i) == 0){
          cv::Rect roi(i-1, j-1, S, S);
          cv::Mat cropped_ra = img_range_bk(roi);
          cv::Mat cropped_re = img_reflec_bk(roi);
          double N_ra = 0;
          double N_re = 0;
          double val_ra = 0;
          double val_re = 0;
          for (int v = 0; v < S; v++){
            for (int u = 0; u < S; u++){
              if (cropped_ra.at<ushort>(v, u) > 0){
                N_ra = N_ra + 1.0; 
                val_ra = val_ra + (double)(cropped_ra.at<ushort>(v, u));
              } 
              if (cropped_re.at<ushort>(v, u) > 0){
                N_re = N_re + 1.0; 
                val_re = val_re + (double)(cropped_re.at<ushort>(v, u));
              } 
            } 
          }
          //std::cout << val / N << std::endl;
          if (N_ra > 0.0) img_range.at<ushort>(j, i) = (ushort)(val_ra / N_ra);
          if (N_re > 0.0) img_reflec.at<ushort>(j, i) = (ushort)(val_re / N_re);
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //// PARSE TO MESSAGE FORMAT
  sensor_msgs::ImagePtr img_range_msg;
  sensor_msgs::ImagePtr img_reflec_msg;
  sensor_msgs::ImagePtr img_x_msg;
  sensor_msgs::ImagePtr img_y_msg;
  sensor_msgs::ImagePtr img_z_msg;
  img_range_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", img_range).toImageMsg();
  img_reflec_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", img_reflec).toImageMsg();
  img_x_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", img_x).toImageMsg();
  img_y_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", img_y).toImageMsg();
  img_z_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", img_z).toImageMsg();
  img_range_msg->header.stamp = scan->header.stamp;
  img_reflec_msg->header.stamp = scan->header.stamp;
  img_x_msg->header.stamp = scan->header.stamp;
  img_y_msg->header.stamp = scan->header.stamp;
  img_z_msg->header.stamp = scan->header.stamp;
  this->img_range_publisher_.publish(img_range_msg);
  this->img_reflec_publisher_.publish(img_reflec_msg);
  this->img_nearir_publisher_.publish(img_reflec_msg);
  this->img_signal_publisher_.publish(img_range_msg);
  this->img_x_publisher_.publish(img_x_msg);
  this->img_y_publisher_.publish(img_y_msg);
  this->img_z_publisher_.publish(img_z_msg);

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  this->alg_.unlock();
  this->pointcloud_mutex_exit();
}

void KittiPreprocessAlgNode::pointcloud_mutex_enter(void)
{
  pthread_mutex_lock(&this->pointcloud_mutex_);
}

void KittiPreprocessAlgNode::pointcloud_mutex_exit(void)
{
  pthread_mutex_unlock(&this->pointcloud_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void KittiPreprocessAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  if(config.rate!=this->getRate())
    this->setRate(config.rate);
  this->config_=config;
  this->alg_.unlock();
}

void KittiPreprocessAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<KittiPreprocessAlgNode>(argc, argv, "kitti_preprocess_alg_node");
}


  /*
  //tf_listener example BEGIN
  try{
    std::string target_frame             = "child_frame";
    std::string source_frame             = "parent_frame";
    ros::Time time                       = ros::Time::now();
    ros::Duration timeout                = ros::Duration(0.1);
    ros::Duration polling_sleep_duration = ros::Duration(0.01);
    this->alg_.unlock();
    bool tf_exists = this->tf_listener_.waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration);
    this->alg_.lock();
    if(tf_exists){
      geometry_msgs::PoseStamped stamped_pose_in;
      stamped_pose_in.header.stamp     = time;
      stamped_pose_in.header.frame_id  = source_frame;
      stamped_pose_in.pose.position.x    = 1.0;
      stamped_pose_in.pose.position.y    = 0.0;
      stamped_pose_in.pose.position.z    = 0.0;
      stamped_pose_in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      ROS_INFO("Original    pose in '%s' frame, with (x,y,z)=[%f,%f,%f], yaw=[%f]", stamped_pose_in.header.frame_id.c_str(), stamped_pose_in.pose.position.x, stamped_pose_in.pose.position.y, stamped_pose_in.pose.position.z, tf::getYaw(stamped_pose_in.pose.orientation));
      geometry_msgs::PoseStamped stamped_pose_out;
      this->tf_listener_.transformPose(target_frame, stamped_pose_in, stamped_pose_out);
      ROS_INFO("Transformed pose in '%s'  frame, with (x,y,z)=[%f,%f,%f], yaw=[%f]", stamped_pose_out.header.frame_id.c_str(), stamped_pose_out.pose.position.x, stamped_pose_out.pose.position.y, stamped_pose_out.pose.position.z, tf::getYaw(stamped_pose_out.pose.orientation));
      ROS_INFO("---");
  

      tf::StampedTransform tf_parent_child;
      tf::Transform tf_parent_point, tf_child_point;
      this->tf_listener_.lookupTransform(source_frame, target_frame, time, tf_parent_child);
      tf_parent_point.setOrigin(tf::Vector3(stamped_pose_in.pose.position.x, stamped_pose_in.pose.position.y, stamped_pose_in.pose.position.z));
      tf_parent_point.setRotation(tf::Quaternion(stamped_pose_in.pose.orientation.x, stamped_pose_in.pose.orientation.y, stamped_pose_in.pose.orientation.z, stamped_pose_in.pose.orientation.w));
      tf_child_point = tf_parent_child.inverse()*tf_parent_point;
      ROS_INFO("Transformed pose in '%s'  frame, with (x,y,z)=[%f,%f,%f], yaw=[%f]", target_frame.c_str(), tf_child_point.getOrigin().x(), tf_child_point.getOrigin().y(), tf_child_point.getOrigin().z(), tf::getYaw(tf_child_point.getRotation()));
      ROS_INFO("---");
    }else{
      ROS_WARN("No transform found from '%s' to '%s'", source_frame.c_str(), target_frame.c_str()); }
  }catch (tf::TransformException &ex){
    ROS_ERROR("TF Exception: %s",ex.what()); }
  ///tf_listener example END
  */

  /*
  //tf_broadcaster example
  this->transform_msg_.header.stamp    = ros::Time::now();
  this->transform_msg_.header.frame_id = "parent_frame";
  this->transform_msg_.child_frame_id  = "child_frame";
  geometry_msgs::Transform t;
  t.translation.x = 0.0;
  t.translation.y = 0.0;
  t.translation.z = 0.0;
  t.rotation = tf::createQuaternionMsgFromYaw(0.0);
  this->transform_msg_.transform = t;
  this->tf_broadcaster_.sendTransform(this->transform_msg_);
  ///tf_broadcaster example
  */
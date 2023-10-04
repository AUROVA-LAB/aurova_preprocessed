#include "kitti_preprocess_alg_node.h"

KittiPreprocessAlgNode::KittiPreprocessAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<KittiPreprocessAlgorithm>()
{
  //init class attributes if necessary
  if(!this->private_node_handle_.getParam("rate", this->config_.rate))
  {
    ROS_WARN("KittiPreprocessAlgNode::KittiPreprocessAlgNode: param 'rate' not found");
  }
  else
    this->setRate(this->config_.rate);

  // [init publishers]
  this->odom_publisher_ = this->private_node_handle_.advertise<nav_msgs::Odometry>("odom", 1);
  
  // [init subscribers]
  this->pointcloud_subscriber_ = this->private_node_handle_.subscribe("/kitti/velo/pointcloud", 1, &KittiPreprocessAlgNode::pointcloud_callback, this);
  pthread_mutex_init(&this->pointcloud_mutex_,NULL);

  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

KittiPreprocessAlgNode::~KittiPreprocessAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->pointcloud_mutex_);
}

void KittiPreprocessAlgNode::mainNodeThread(void)
{
  //lock access to algorithm if necessary
  this->alg_.lock();
  ROS_DEBUG("KittiPreprocessAlgNode::mainNodeThread");
  // [fill msg structures]
  // Initialize the topic message structure
  //this->odom_Odometry_msg_.data = my_var;

  
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

  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->odom_publisher_.publish(this->odom_Odometry_msg_);

  
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

  
  this->alg_.unlock();
}

/*  [subscriber callbacks] */
void KittiPreprocessAlgNode::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("KittiPreprocessAlgNode::pointcloud_callback: New Message Received");
  
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->pointcloud_mutex_enter();

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->pointcloud_mutex_exit();
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

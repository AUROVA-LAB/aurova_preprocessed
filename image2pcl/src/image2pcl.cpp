#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <math.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <detection_msgs/BoundingBox.h>
#include <detection_msgs/BoundingBoxes.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <limits>
#include <chrono> 

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
visualization_msgs::Marker marker;

//Publisher
ros::Publisher pc_filtered_pub; // publisher de la imagen de puntos filtrada
ros::Publisher pc_raw_pub; // publisher pc

// input topics 
std::string xTopic  = "/ouster/x_image";
std::string yTopic  = "/ouster/y_image";
std::string zTopic  = "/ouster/z_image";
std::string rangeTopic  = "/ouster/range_image";
std::string maskTopic = "/mask/topic";
std::string outTopicPc = "/out/topic_pc";
std::string outTopicDt = "/out/topic_dt";
bool include_detections = true;
bool include_pc = true;

///////////////////////////////////////callback


void callback_dt(const ImageConstPtr& in_image, const ImageConstPtr& in_mask)
{
  cv_bridge::CvImagePtr cv_range, cv_mask;
  try
  {
    cv_range = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::MONO16);
    cv_mask = cv_bridge::toCvCopy(in_mask, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img_range  = cv_range->image; // get image matrix of cv_range
  cv::Mat img_mask  = cv_mask->image;   // get image matrix of cv_range

  Eigen::Matrix<float,Dynamic,Dynamic> depth_data , data_metrics, data_mask;// matrix with image values and matrix qith image values into real range data
  cv2eigen(img_range,depth_data);       // convert img_range into eigen matrix
  cv2eigen(img_mask,data_mask); 
  data_metrics = depth_data*(261/pow(2,16)); // resolution 16 bits -> 4mm. 
  
 
  PointCloud::Ptr point_cloud (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  point_cloud->width = img_range.cols; 
  point_cloud->height = img_range.rows;
  point_cloud->is_dense = false;
  point_cloud->points.resize (point_cloud->width * point_cloud->height);
  uint num_pix = 0;

  for (int i = 0;i<img_range.rows; i++){
      for (int j = 0;j<img_range.cols; j++){

        if (data_metrics(i,j)==0 || data_mask(i,j)==0)
          continue;

        float ang_h = 22.5 - (45.0/128.0)*i;
        ang_h = ang_h*M_PI/180.0;
        float ang_w = 184.0 - (360.0/2048.0)*j;
        ang_w = ang_w*M_PI/180.0;

        float z = data_metrics(i,j) * sin(ang_h);
        float y = sqrt(pow(data_metrics(i,j),2)-pow(z,2))*sin(ang_w);
        float x = sqrt(pow(data_metrics(i,j),2)-pow(z,2))*cos(ang_w);
        //asignacion de valores maximo y minimos

        point_cloud->points[num_pix].x = x;
        point_cloud->points[num_pix].y = y;
        point_cloud->points[num_pix].z = z;
        cloud_out->push_back(point_cloud->points[num_pix]); 
        num_pix++; 

      }
  } 

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud_out);
  vg.setLeafSize(0.5f, 0.5f, 100.0f);
  vg.filter(*cloud_out);
  
  cloud_out->is_dense = false;
  cloud_out->width = (int) cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->header.frame_id = "os_sensor";
  ros::Time time_st = cv_mask->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  cloud_out->header.stamp = time_st.toNSec()/1e3;
  cloud_out->header.seq = in_mask->header.seq;
  if (include_detections) pc_filtered_pub.publish (cloud_out);
  

}

void callback_pc(const ImageConstPtr& in_image)
{
  cv_bridge::CvImagePtr cv_range;
  try
  {
    cv_range = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::MONO16);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img_range  = cv_range->image; // get image matrix of cv_range

  Eigen::Matrix<float,Dynamic,Dynamic> depth_data , data_metrics;// matrix with image values and matrix qith image values into real range data
  cv2eigen(img_range,depth_data);       // convert img_range into eigen matrix
  data_metrics = depth_data*(261/pow(2,16)); // resolution 16 bits -> 4mm. 
  
 
  PointCloud::Ptr point_cloud (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  point_cloud->width = img_range.cols; 
  point_cloud->height = img_range.rows;
  point_cloud->is_dense = false;
  point_cloud->points.resize (point_cloud->width * point_cloud->height);
  uint num_pix = 0;

  for (int i = 0;i<img_range.rows; i++){
      for (int j = 0;j<img_range.cols; j++){

        if (data_metrics(i,j)==0)
          continue;

        float ang_h = 22.5 - (45.0/128.0)*i;
        ang_h = ang_h*M_PI/180.0;
        float ang_w = 184.0 - (360.0/2048.0)*j;
        ang_w = ang_w*M_PI/180.0;

        float z = data_metrics(i,j) * sin(ang_h);
        float y = sqrt(pow(data_metrics(i,j),2)-pow(z,2))*sin(ang_w);
        float x = sqrt(pow(data_metrics(i,j),2)-pow(z,2))*cos(ang_w);
        //asignacion de valores maximo y minimos

        point_cloud->points[num_pix].x = x;
        point_cloud->points[num_pix].y = y;
        point_cloud->points[num_pix].z = z;
        cloud_out->push_back(point_cloud->points[num_pix]); 
        num_pix++; 

      }
  } 
  
  cloud_out->is_dense = false;
  cloud_out->width = (int) cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->header.frame_id = "os_sensor";
  ros::Time time_st = cv_range->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  cloud_out->header.stamp = time_st.toNSec()/1e3;
  
  if (include_pc) pc_raw_pub.publish (cloud_out);
  

}

//// From PCL backup to image
void callback_dt(const ImageConstPtr& x_image, const ImageConstPtr& y_image, const ImageConstPtr& z_image, const ImageConstPtr& in_mask)
{
  cv_bridge::CvImagePtr cv_x, cv_y, cv_z, cv_mask;
  try
  {
    cv_x = cv_bridge::toCvCopy(x_image, sensor_msgs::image_encodings::MONO16);
    cv_y = cv_bridge::toCvCopy(y_image, sensor_msgs::image_encodings::MONO16);
    cv_z = cv_bridge::toCvCopy(z_image, sensor_msgs::image_encodings::MONO16);
    cv_mask = cv_bridge::toCvCopy(in_mask, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img_x  = cv_x->image; // get image matrix of cv_x
  cv::Mat img_y  = cv_y->image;
  cv::Mat img_z  = cv_z->image;
  cv::Mat img_mask  = cv_mask->image;

  Eigen::Matrix<float,Dynamic,Dynamic> data_mask;// matrix with image values and matrix qith image values into real range data
  cv2eigen(img_mask,data_mask); 

  PointCloud::Ptr point_cloud (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  point_cloud->width = img_x.cols; 
  point_cloud->height = img_x.rows;
  point_cloud->is_dense = false;
  point_cloud->points.resize (point_cloud->width * point_cloud->height);
  float max_range = 100.0;
  uint num_pix = 0;

  for (int i = 0; i < img_x.rows; i++){
      for (int j = 0; j < img_x.cols; j++){

        if (data_mask(i, j) == 0)
          continue;

        if (img_x.at<ushort>(i, j) > 0){
          double x = ((double)(img_x.at<ushort>(i, j)) / pow(2,16)) * 2 * max_range - max_range;
          double y = ((double)(img_y.at<ushort>(i, j)) / pow(2,16)) * 2 * max_range - max_range;
          double z = ((double)(img_z.at<ushort>(i, j)) / pow(2,16)) * 2 * max_range - max_range;

          point_cloud->points[num_pix].x = x;
          point_cloud->points[num_pix].y = y;
          point_cloud->points[num_pix].z = z;
          cloud_out->push_back(point_cloud->points[num_pix]); 
          num_pix++; 
        }
      }
  } 
  
  cloud_out->is_dense = false;
  cloud_out->width = (int) cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->header.frame_id = "velo_link";
  ros::Time time_st = cv_mask->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  cloud_out->header.stamp = time_st.toNSec()/1e3;
  cloud_out->header.seq = in_mask->header.seq;
  pc_filtered_pub.publish (cloud_out);

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image2pcl");
  ros::NodeHandle nh("~");  
  std::cout<<"Nodo image2pcl inicializado: "<<std::endl;
  
  /// Load Parameters

  nh.getParam("range_img", rangeTopic);
  nh.getParam("mask_img", maskTopic);
  nh.getParam("out_pc", outTopicPc);
  nh.getParam("out_detections", outTopicDt);
  nh.getParam("include_detections", include_detections);
  nh.getParam("include_pc", include_pc);

  message_filters::Subscriber<Image> x_sub(nh, xTopic , 1);
  message_filters::Subscriber<Image> y_sub(nh, yTopic , 1);
  message_filters::Subscriber<Image> z_sub(nh, zTopic , 1);
  message_filters::Subscriber<Image> range_sub (nh, rangeTopic,  1);
  message_filters::Subscriber<Image> mask_sub(nh, maskTopic , 1);
  ros::Subscriber range_aux_sub = nh.subscribe<Image>(rangeTopic, 1, callback_pc);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), range_sub, mask_sub);
  sync.registerCallback(boost::bind(&callback_dt, _1, _2));

  typedef sync_policies::ApproximateTime<Image, Image, Image,Image> MySyncPolicy_bk;
  Synchronizer<MySyncPolicy_bk> sync_bk(MySyncPolicy_bk(1), x_sub, y_sub, z_sub, mask_sub);
  sync_bk.registerCallback(boost::bind(&callback_dt, _1, _2, _3, _4));

  pc_filtered_pub = nh.advertise<PointCloud> (outTopicDt, 1);  
  pc_raw_pub = nh.advertise<PointCloud> (outTopicPc, 1);

  ros::spin();
}

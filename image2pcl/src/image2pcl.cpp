#define PCL_NO_PRECOMPILE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
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
#include <pcl/pcl_macros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <math.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <detection_msgs/msg/bounding_box.hpp>
#include <detection_msgs/msg/bounding_boxes.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <visualization_msgs/msg/marker.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <limits>
#include <chrono> 

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

// The camera calibration needed the ring channel, but it was prepared for PointXYZIR, so intensity channel was also added (but not filled)
struct PointXYZIR
{
  PCL_ADD_POINT4D;
  float intensity;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,           // here we assume a XYZ + ring (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint16_t, ring, ring))

typedef pcl::PointCloud<PointXYZIR> PointCloud;
visualization_msgs::msg::Marker marker;

//Publisher
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_filtered_pub; // publisher de la imagen de puntos filtrada
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_raw_pub; // publisher pc
rclcpp::Node::SharedPtr nh;

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


void callback_dt_2(const sensor_msgs::msg::Image::ConstSharedPtr&  in_image, const sensor_msgs::msg::Image::ConstSharedPtr&  in_mask)
{
  cv_bridge::CvImagePtr cv_range, cv_mask;
  try
  {
    cv_range = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::MONO16);
    cv_mask = cv_bridge::toCvCopy(in_mask, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(nh->get_logger(),"cv_bridge exception: %s", e.what());
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

  pcl::VoxelGrid<PointXYZIR> vg;
  vg.setInputCloud(cloud_out);
  vg.setLeafSize(0.5f, 0.5f, 100.0f);
  vg.filter(*cloud_out);
  
  cloud_out->is_dense = false;
  cloud_out->width = (int) cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->header.frame_id = "os_sensor";
  rclcpp::Time time_st = cv_mask->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  cloud_out->header.stamp = time_st.nanoseconds()/1e3;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_out, cloud_msg);
  if (include_detections) pc_filtered_pub->publish (cloud_msg);
  

}

void callback_pc(const sensor_msgs::msg::Image::SharedPtr  in_image)
{
  cv_bridge::CvImagePtr cv_range;
  try
  {
    cv_range = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::MONO16);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(nh->get_logger(),"cv_bridge exception: %s", e.what());
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
        point_cloud->points[num_pix].ring = img_range.rows-i-1;
        cloud_out->push_back(point_cloud->points[num_pix]); 
        num_pix++; 

      }
  } 
  
  cloud_out->is_dense = false;
  cloud_out->width = (int) cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->header.frame_id = "os_sensor";
  rclcpp::Time time_st = cv_range->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  cloud_out->header.stamp = time_st.nanoseconds()/1e3;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_out, cloud_msg);
  if (include_pc) pc_raw_pub->publish (cloud_msg);
  

}

//// From PCL backup to image
void callback_dt_4(const sensor_msgs::msg::Image::ConstSharedPtr&  x_image, const sensor_msgs::msg::Image::ConstSharedPtr&  y_image, const sensor_msgs::msg::Image::ConstSharedPtr&  z_image, const sensor_msgs::msg::Image::ConstSharedPtr&  in_mask)
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
    RCLCPP_ERROR(nh->get_logger(),"cv_bridge exception: %s", e.what());
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
  rclcpp::Time time_st = cv_mask->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  cloud_out->header.stamp = time_st.nanoseconds()/1e3;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_out, cloud_msg);
  pc_filtered_pub->publish (cloud_msg);
}

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  nh = std::make_shared<rclcpp::Node>("image2pcl");

  std::cout<<"Nodo image2pcl inicializado: "<<std::endl;
  
  /// Load Parameters
  nh->declare_parameter("range_img",rangeTopic);
  nh->get_parameter("range_img", rangeTopic);
  nh->declare_parameter("mask_img", maskTopic);
  nh->get_parameter("mask_img", maskTopic);
  nh->declare_parameter("out_pc", outTopicPc);
  nh->get_parameter("out_pc", outTopicPc);
  nh->declare_parameter("out_detections", outTopicDt);
  nh->get_parameter("out_detections", outTopicDt);
  nh->declare_parameter("include_detections", include_detections);
  nh->get_parameter("include_detections", include_detections);
  nh->declare_parameter("include_pc", include_pc);
  nh->get_parameter("include_pc", include_pc);


  message_filters::Subscriber<sensor_msgs::msg::Image> x_sub;
  x_sub.subscribe(nh, xTopic);
  message_filters::Subscriber<sensor_msgs::msg::Image>  y_sub;
  y_sub.subscribe(nh, yTopic);
  message_filters::Subscriber<sensor_msgs::msg::Image> z_sub;
  z_sub.subscribe(nh, zTopic);
  message_filters::Subscriber<sensor_msgs::msg::Image> range_sub;
  range_sub.subscribe(nh, rangeTopic);
  message_filters::Subscriber<sensor_msgs::msg::Image> mask_sub;
  mask_sub.subscribe(nh, maskTopic);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr range_aux_sub = nh->create_subscription<sensor_msgs::msg::Image>(rangeTopic, 1, std::bind(&callback_pc, std::placeholders::_1));

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  auto sync = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(1), range_sub, mask_sub);
  sync->registerCallback(std::bind(&callback_dt_2, std::placeholders::_1, std::placeholders::_2));

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy_bk;
  auto sync_bk = std::make_shared<message_filters::Synchronizer<MySyncPolicy_bk>>(MySyncPolicy_bk(1), x_sub, y_sub, z_sub, mask_sub);
  sync_bk->registerCallback(std::bind(&callback_dt_4, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  pc_filtered_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2> (outTopicDt, 1);  
  pc_raw_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2> (outTopicPc, 1);

  rclcpp::spin(nh);
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.hpp>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgD_pub;
std::shared_ptr<pcl::RangeImageSpherical> rngSpheric;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

rclcpp::Node::SharedPtr nh;

float maxlen =100;
float minlen = 0.1;
float angular_resolution_x = 0.25f;
float angular_resolution_y = 2.05f;
float max_angle_width= 360.0f;
float max_angle_height = 360.0f;

float z_max = 100.0f;
float z_min = 100.0f;

std::string pcTopic = "/velodyne_points";


void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointCloud)
{
  if (msg_pointCloud == NULL) return;

  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr aux (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  std::vector<int> indices;
  pcl::fromROSMsg(*msg_pointCloud, *cloud_in);
  pcl::removeNaNFromPointCloud(*cloud_in, *aux, indices);


  for (int i = 0; i < (int) aux->points.size(); i++)
  {
      double distance = sqrt(pow(aux->points[i].x,2)+pow(aux->points[i].y,2)+pow(aux->points[i].z,2));
      if(distance<minlen || distance>maxlen)
          continue;
     cloud_out->push_back(aux->points[i]);          
  }

  rngSpheric->pcl::RangeImage::createFromPointCloud(*cloud_out, pcl::deg2rad(angular_resolution_x), pcl::deg2rad(angular_resolution_y),
                                       pcl::deg2rad(max_angle_width), pcl::deg2rad(max_angle_height),
                                       Eigen::Affine3f::Identity(), coordinate_frame, 0.0f, 0.0f, 0);
                                

  
  int cols = (rngSpheric->width) * 2;
  int rows = rngSpheric->height;
  

  cv::Mat dephtImage =  cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));

  unsigned short range = 0 , z_range = 0;

  for (int i=0; i< int(cols/2); ++i)
   {
       
      for (int j=0; j<rows ; ++j)
      {
        float r =  rngSpheric->getPoint(i, j).range;     
        float zz = rngSpheric->getPoint(i, j).z + 50.0;          
        
        if(std::isinf(r) || r<minlen || r>maxlen || std::isnan(zz)){

            continue;
        }

        
        range = 1-(pow(2,16)/ (maxlen - minlen))*(r-minlen);   
        dephtImage.at<ushort>(j, i) = range;       
        z_range = (zz/z_max)* pow(2,16);

        dephtImage.at<ushort>(j, i+int(cols/2)) = z_range;
              
    }
  }

  sensor_msgs::msg::Image::SharedPtr image_msg;
  image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", dephtImage).toImageMsg();

  image_msg->header.frame_id = msg_pointCloud->header.frame_id;
  image_msg->header.stamp = msg_pointCloud->header.stamp;
  imgD_pub->publish(*image_msg);

}

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  nh = std::make_shared<rclcpp::Node>("pontCloud2dephtImage");


  nh->declare_parameter("maxlen",maxlen);
  nh->get_parameter("maxlen", maxlen);
  nh->declare_parameter("minlen",minlen);
  nh->get_parameter("minlen", minlen);
  nh->declare_parameter("angular_resolution_x",angular_resolution_x);
  nh->get_parameter("angular_resolution_x", angular_resolution_x);
  nh->declare_parameter("angular_resolution_y",angular_resolution_y);
  nh->get_parameter("angular_resolution_y", angular_resolution_y);
  nh->declare_parameter("max_angle_width",max_angle_width);
  nh->get_parameter("max_angle_width", max_angle_width);
  nh->declare_parameter("max_angle_height",max_angle_height);
  nh->get_parameter("max_angle_height", max_angle_height);
  nh->declare_parameter("pcTopic",pcTopic);
  nh->get_parameter("pcTopic", pcTopic);


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(pcTopic, 10, std::bind(&callback, std::placeholders::_1));
  rngSpheric = std::shared_ptr<pcl::RangeImageSpherical>(new pcl::RangeImageSpherical);
  imgD_pub = nh->create_publisher<sensor_msgs::msg::Image>("/depth_z_image"  , 10);  

  rclcpp::spin(nh);
  return 0;
}

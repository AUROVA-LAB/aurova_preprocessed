#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


ros::Publisher imgD_pub;
boost::shared_ptr<pcl::RangeImageSpherical> rngSpheric;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

float maxlen =100;
float minlen = 0.1;
float angular_resolution_x = 0.25f;
float angular_resolution_y = 2.05f;
float max_angle_width= 360.0f;
float max_angle_height = 360.0f;

float z_max = 100.0f;
float z_min = 100.0f;

std::string pcTopic = "/velodyne_points";


void callback(const PointCloud::ConstPtr& msg_pointCloud)
{
  if (msg_pointCloud == NULL) return;

  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*msg_pointCloud, *cloud_in, indices);

  for (int i = 0; i < (int) cloud_in->points.size(); i++)
  {
      double distance = sqrt(cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y);
      if(distance<minlen || distance>maxlen)
          continue;
     cloud_out->push_back(cloud_in->points[i]);

     
  }

  rngSpheric->pcl::RangeImage::createFromPointCloud(*cloud_out, pcl::deg2rad(angular_resolution_x), pcl::deg2rad(angular_resolution_y),
                                       pcl::deg2rad(max_angle_width), pcl::deg2rad(max_angle_height),
                                       Eigen::Affine3f::Identity(), coordinate_frame, 0.0f, 0.0f, 0);
                                
  rngSpheric->header.frame_id = msg_pointCloud->header.frame_id;
  rngSpheric->header.stamp    = msg_pointCloud->header.stamp;

  
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

  sensor_msgs::ImagePtr image_msg;
  image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", dephtImage).toImageMsg();

  image_msg->header = pcl_conversions::fromPCL(msg_pointCloud->header);
  imgD_pub.publish(image_msg);

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "pontCloud2dephtImage");
  ros::NodeHandle nh;  

  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/angular_resolution_x", angular_resolution_x);
  nh.getParam("/angular_resolution_y", angular_resolution_y);
  nh.getParam("/max_angle_width", max_angle_width);
  nh.getParam("/max_angle_height", max_angle_height);
  nh.getParam("/pcTopic", pcTopic);


  ros::Subscriber sub = nh.subscribe<PointCloud>(pcTopic, 10, callback);
  rngSpheric = boost::shared_ptr<pcl::RangeImageSpherical>(new pcl::RangeImageSpherical);
  imgD_pub = nh.advertise<sensor_msgs::Image>("/depth_z_image", 10);

  ros::spin();
  return 0;
}

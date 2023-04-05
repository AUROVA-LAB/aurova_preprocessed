#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <std_msgs/Float64.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <math.h>
#include <iostream>
#include <limits>
#include <chrono>
#include <Eigen/Dense>


using namespace Eigen;
using namespace sensor_msgs;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher edgeImg_pub, rangImg_pub, grndImg_pub, surfImg_pub;
ros::Publisher edgePCL_pub;
ros::Publisher surfPCL_pub;
ros::Publisher grndPCL_pub;
ros::Publisher time_average;

boost::shared_ptr<pcl::RangeImageSpherical> rngSpheric;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

// initial parameters
float maxlen =100;
float minlen = 3.0;
float angular_resolution_x = 0.5f;
float angular_resolution_y = 0.5f;
float max_angle_width= 360.0f;
float max_angle_height = 180.0f;
bool voxel = false;
bool grndSurf = false;

float z_max = std::numeric_limits<float>::min();
float z_min = std::numeric_limits<float>::max();
float r_max = std::numeric_limits<float>::min();
float r_min = std::numeric_limits<float>::max();

typedef std::chrono::high_resolution_clock Clock;

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
      double distance = sqrt(pow(cloud_in->points[i].x,2)+pow(cloud_in->points[i].y,2)+pow(cloud_in->points[i].z,2));
      if(distance<minlen || distance>maxlen)
          continue;
     cloud_out->push_back(cloud_in->points[i]);          
  }
  

  ///////////////////////Rotacion lidar: pruebas del lidar velodyne que esta girado -17 grados
  /*Eigen::Matrix<float, 3, 3> R;
  R << 0.9563047,  0.0000000,  0.2923717,
   0.0000000,  1.0000000,  0.0000000,
  -0.2923717,  0.0000000,  0.9563047;*/

  Eigen::Affine3f aff = Eigen::Affine3f::Identity();
  //aff.rotate (Eigen::AngleAxisf (-0*M_PI/180.0, Eigen::Vector3f::UnitY()));

  /////////////////////////////////////////////77

  rngSpheric->pcl::RangeImage::createFromPointCloud(*cloud_out, pcl::deg2rad(angular_resolution_x), pcl::deg2rad(angular_resolution_y),
                                       pcl::deg2rad(max_angle_width), pcl::deg2rad(max_angle_height),
                                       aff, coordinate_frame, 0.0f, 0.0f, 0);
  auto t1 = Clock::now(); // lectura de tiempo para estimar el retardo del proceso                              
  rngSpheric->header.frame_id = msg_pointCloud->header.frame_id;
  rngSpheric->header.stamp    = msg_pointCloud->header.stamp;
  
  int cols = rngSpheric->width;
  int rows = rngSpheric->height;
  
  cv::Mat dImage =  cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));
  cv::Mat xImage =  cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));
  cv::Mat yImage =  cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));
  cv::Mat zImage =  cv::Mat::zeros(rows, cols, cv_bridge::getCvType("mono16"));

  for (int i=0; i< int(cols); ++i)        
      for (int j=0; j<rows ; ++j)
      {        
        float r =  rngSpheric->getPoint(i, j).range; 
        float zz = rngSpheric->getPoint(i, j).z;   

        if(std::isinf(r) || r<minlen || r>maxlen || std::isnan(zz))
            continue;

        if (zz >z_max ) z_max = zz;     
        if (r  >r_max ) r_max = r; 
        if (zz <z_min ) z_min = zz; 
        if (r  <r_min ) r_min = r; 

        dImage.at<ushort>(j,i) = 1-(pow(2,16)/ (r_max - r_min))*(r -r_min);    
        zImage.at<ushort>(j,i) = 1-(pow(2,16)/ (z_max - z_min))*(zz-z_min);                     
      }
  

////////////////////////////////////////////// Filtrado por imagen//////////////////////////////////  

  cv::Mat img_frame_dep = dImage.clone();
  img_frame_dep.convertTo(img_frame_dep, CV_8UC1, 1 / 256.0);

  ////////////////////////////////////////ground filter ///////////////////////////////
  cv::Mat grad_x, abs_grad_x ,th_sobel;
  cv::Sobel(img_frame_dep, grad_x, CV_64F, 0, 1, 1);
  cv::convertScaleAbs(grad_x, abs_grad_x);
  cv::threshold(abs_grad_x,th_sobel,1, 255, cv::THRESH_BINARY_INV);
  cv::Mat ground_mask1;  
  int image_data[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  cv::Mat element = cv::Mat(3, 3, CV_32F, image_data);  
  cv::morphologyEx(th_sobel, ground_mask1,cv::MORPH_CLOSE, element,cv::Point(1,1),1);
  
  cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
  cv::morphologyEx(ground_mask1, ground_mask1, cv::MORPH_CLOSE,  close_kernel,cv::Point(1,1),5);
  //

  // Edge 

  cv::Mat edge_y, abs_edge_y ,th_edge, edge_frame;
  cv::Mat ground_mask2 = ground_mask1/255.0;
  edge_frame = img_frame_dep.mul((ground_mask2));
  cv::Sobel(edge_frame, edge_y, CV_64F, 1, 0, 1);
  cv::convertScaleAbs(edge_y, abs_edge_y);
  cv::threshold(abs_edge_y,th_edge,5, 255, cv::THRESH_BINARY);
  cv::Mat curr_edge = dImage.clone();
  
  th_edge.convertTo(th_edge, CV_16U, 1 / 255.0);  
  curr_edge = curr_edge.mul((th_edge));

  //ground segmentation
  cv::Mat ground_mask_inv;
  cv::Mat curr_ground = dImage.clone();

  cv::bitwise_not(ground_mask1, ground_mask_inv);
  ground_mask_inv = ground_mask_inv/255.0;
  ground_mask_inv.convertTo(ground_mask_inv, CV_16U);  
  curr_ground = curr_ground.mul((ground_mask_inv)) ; 
  
  //surf segmentation
  cv::Mat curr_surf = dImage.clone();
  curr_surf = curr_surf - (curr_ground+curr_edge);

/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////point cloud reconstruction///////////////////////////////////////////

// point cloud reconstruction
  cv::Mat pc_surf, pc_edge;

   PointCloud::Ptr edge_cloud (new PointCloud);
  edge_cloud->width  = dImage.cols; 
  edge_cloud->height = dImage.rows;
  edge_cloud->is_dense = false;
  edge_cloud->points.resize (edge_cloud->width * edge_cloud->height);

  PointCloud::Ptr surf_cloud (new PointCloud);
  surf_cloud->width  = dImage.cols; 
  surf_cloud->height = dImage.rows;
  surf_cloud->is_dense = false;
  surf_cloud->points.resize (surf_cloud->width * surf_cloud->height);

  PointCloud::Ptr ground_cloud (new PointCloud);
  ground_cloud->width  = dImage.cols; 
  ground_cloud->height = dImage.rows;
  ground_cloud->is_dense = false;
  ground_cloud->points.resize (ground_cloud->width * ground_cloud->height);

  Eigen::Matrix<float,Dynamic,Dynamic> edge_sz, surf_sz, ground_sz, z_eigen;
  cv2eigen(curr_edge,edge_sz);  
  cv2eigen(curr_surf,surf_sz);
  cv2eigen(curr_ground,ground_sz);
  cv2eigen(zImage,z_eigen);

  int numpe = 0, numps =0 , numpg = 0;
  for (int i=0; i< dImage.rows; i+=1)
   {       
      for (int j=0; j<dImage.cols ; j+=1)
      {

        float ang= M_PI - (((max_angle_width/180.0) * M_PI * j )/(dImage.cols));

        float z_position = (pow(2,16)-z_eigen(i,j)) *((z_max - z_min)/ pow(2,16)) + z_min;   
        
        if(!(edge_sz(i,j)== 0 || edge_sz(i,j) > (pow(2,16)- (r_min/r_max) * pow(2,16)))){
       
        float edge_data  = (pow(2,16)-edge_sz(i,j))*((r_max - r_min)/ pow(2,16)) + r_min ;                      
        float edge_x_data = sqrt(pow(edge_data,2)- pow(z_position,2)) * cos(ang);
        float edge_y_data = sqrt(pow(edge_data,2)- pow(z_position,2)) * sin(ang);

        edge_cloud->points[numpe].x = edge_x_data;
        edge_cloud->points[numpe].y = edge_y_data;
        edge_cloud->points[numpe].z = z_position;
        numpe++;
        }
        else{
          
          curr_edge.at<ushort>(i, j) = 0;         
        }

        if(!(surf_sz(i,j)== 0 || surf_sz(i,j) > (pow(2,16)- (r_min/r_max) * pow(2,16)))){

          float surf_data = (pow(2,16)-surf_sz(i,j))*((r_max - r_min )/ pow(2,16))+r_min ;        
          float surf_x_data = sqrt(pow(surf_data,2)- pow(z_position,2)) * cos(ang);
          float surf_y_data = sqrt(pow(surf_data,2)- pow(z_position,2)) * sin(ang);
          
          surf_cloud->points[numps].x = surf_x_data;
          surf_cloud->points[numps].y = surf_y_data;
          surf_cloud->points[numps].z = z_position;

          numps++;
        }
        else{
          curr_surf.at<ushort>(i, j) = 0;
        }

        if(!(ground_sz(i,j)== 0 || ground_sz(i,j) > (pow(2,16)- (r_min/r_max) * pow(2,16)))){
        
          float ground_data = (pow(2,16)-ground_sz(i,j))*((r_max - r_min)/pow(2,16))+r_min; 
          float groundx_data = sqrt(pow(ground_data,2)- pow(z_position,2)) * cos(ang);
          float groundy_data = sqrt(pow(ground_data,2)- pow(z_position,2)) * sin(ang);       

          if (!grndSurf){
            ground_cloud->points[numpg].x = groundx_data;
            ground_cloud->points[numpg].y = groundy_data;
            ground_cloud->points[numpg].z = z_position;
            numpg++;     
          }     
          else{
            surf_cloud->points[numps].x = groundx_data;
            surf_cloud->points[numps].y = groundy_data;
            surf_cloud->points[numps].z = z_position;
            numps++;
          }
        }
        else{
          curr_ground.at<ushort>(i, j) = 0;
        }
      }
   }  

  // Remuestrear la nube de puntos para acelerar el procesamiento
  if(voxel){
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(ground_cloud);
    vg.setLeafSize(0.5f, 0.5f, 0.5f);
    vg.filter(*ground_cloud);

    vg.setInputCloud(surf_cloud);
    vg.setLeafSize(1.0f, 1.0f, 1.0f);
    vg.filter(*surf_cloud);

    vg.setInputCloud(edge_cloud);
    vg.setLeafSize(0.5f, 0.5f, 0.1f);
    vg.filter(*edge_cloud);
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////

  edge_cloud->header   =   msg_pointCloud->header;
  ground_cloud->header = msg_pointCloud->header;
  surf_cloud->header   =   msg_pointCloud->header;

  edgePCL_pub.publish (edge_cloud);
  surfPCL_pub.publish (surf_cloud);
  grndPCL_pub.publish (ground_cloud);

  sensor_msgs::ImagePtr edgeImg_msg, rangImg_msg, grndImg_msg, surfImg_msg;
  edgeImg_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", curr_edge).toImageMsg();  
  surfImg_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", curr_surf).toImageMsg();
  grndImg_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", curr_ground).toImageMsg();
  rangImg_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", dImage).toImageMsg();

  edgeImg_msg->header = pcl_conversions::fromPCL(msg_pointCloud->header);
  rangImg_msg->header = pcl_conversions::fromPCL(msg_pointCloud->header);
  grndImg_msg->header = pcl_conversions::fromPCL(msg_pointCloud->header);
  surfImg_msg->header = pcl_conversions::fromPCL(msg_pointCloud->header);

  edgeImg_pub.publish(edgeImg_msg);
  rangImg_pub.publish(rangImg_msg);
  grndImg_pub.publish(grndImg_msg);
  surfImg_pub.publish(surfImg_msg);
  auto t4= Clock::now();
  float time_t = std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t1).count()/1000000000.0;
  std::cout<<"Time_per_frame: "<<time_t<<std::endl;
  std_msgs::Float64 time_msg;
  time_msg.data = time_t*1000.0;
  time_average.publish(time_msg);


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
  nh.getParam("/voxel",voxel);
  nh.getParam("/groundSurf_fusion",grndSurf);

  
  ros::Subscriber sub = nh.subscribe<PointCloud>(pcTopic, 10, callback);
  rngSpheric = boost::shared_ptr<pcl::RangeImageSpherical>(new pcl::RangeImageSpherical);
  edgeImg_pub = nh.advertise<sensor_msgs::Image>("/pclImage/edge"  , 10);  
  surfImg_pub = nh.advertise<sensor_msgs::Image>("/pclImage/surf"  , 10);
  grndImg_pub = nh.advertise<sensor_msgs::Image>("/pclImage/ground", 10);
  rangImg_pub = nh.advertise<sensor_msgs::Image>("/pclImage/range" , 10);

  edgePCL_pub= nh.advertise<PointCloud>  ("/pcl_edge"  ,10);
  surfPCL_pub = nh.advertise<PointCloud> ("/pcl_surf"  ,10);  
  grndPCL_pub = nh.advertise<PointCloud> ("/pcl_ground",10);
  time_average = nh.advertise<std_msgs::Float64>("/time_feature_average", 1);

  ros::spin();
  return 0;
}

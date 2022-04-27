#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <std_msgs/Header.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>

#include <chrono>

#include <sensor_msgs/PointCloud2.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

#include <math.h>

using namespace Eigen;
using namespace sensor_msgs;
using namespace std;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub_img;
ros::Publisher pub_edge;
ros::Publisher pub_surf;
ros::Publisher pub_ground;
ros::Publisher pub_1;
ros::Publisher pub_2;
ros::Publisher pub_3;

float max_depth =100.0;
float min_depth = 8.0;
std::string imgTopic = "/depht_image";

int total_frame=0;
float time_delay  = 0;
double total_time =0;

void callback(const ImageConstPtr& imgIn)
{

  std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();




    cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(imgIn, sensor_msgs::image_encodings::MONO16);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

  /*cv::Mat resize = cv_ptr->image; 
  cv::Mat img;
  cv::resize(resize, img, cv::Size(2048*2, 80), cv::INTER_LINEAR);*/
  
  cv::Mat img  = cv_ptr->image;  

  int cols = img.cols;
  int rows = img.rows;

  cv::Rect roi;
  roi.x = 0;
  roi.y = 0;
  roi.width = cols/2;
  roi.height = rows;
  cv::Mat l_img = img(roi);
  
  roi.x = cols/2 ;
  cv::Mat z_img = img(roi);

  Eigen::Matrix<float,Dynamic,Dynamic> z_data;
  Eigen::Matrix<float,Dynamic,Dynamic> z_range = Eigen::Matrix<float,Dynamic,Dynamic>::Zero(rows, cols/2);;
  
  //Eigen::Matrix<float,Dynamic,Dynamic> offset_z = Eigen::Matrix<float,Dynamic,Dynamic>::Ones(rows, cols/2);
  
  cv2eigen(z_img,z_data); 

  for (int i=0; i< z_data.rows(); ++i)
   {       
      for (int j=0; j<z_data.cols() ; ++j)
      {
        if (z_data(i,j) == 0)
          continue; 

        z_range(i,j) = (100.0/pow(2,16))*(z_data(i,j)) -50.0; 
      }
   }
 
  //std::cout << "Here is the matrix m:\n" << z_range << std::endl;


  cv::Mat img_frame_eq;

  cv::Mat img_frame = l_img.clone();
  img_frame.convertTo(img_frame, CV_8UC1, 1 / 256.0);
  cv::equalizeHist(img_frame, img_frame_eq); 

//ground filter
  cv::Mat grad_x, abs_grad_x ,th_sobel;
  cv::Sobel(img_frame_eq, grad_x, CV_64F, 0, 1, 1);
  cv::convertScaleAbs(grad_x, abs_grad_x);

  cv::threshold(abs_grad_x,th_sobel,1, 255, cv::THRESH_BINARY_INV);

  cv::Mat ground_mask1, ground_mask2;
  int image_data[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  cv::Mat element = cv::Mat(3, 3, CV_32F, image_data);  
  cv::morphologyEx(th_sobel, ground_mask1,cv::MORPH_CLOSE, element,cv::Point(1,1),1);

// Edge 
  cv::Mat edge_y, abs_edge_y ,th_edge, edge_frame;
 
  ground_mask2 = ground_mask1/255.0;
  //std::cout<<"Datos"<<cl_sobel<<endl;
 
  edge_frame = img_frame.mul((ground_mask2));

  cv::Sobel(edge_frame, edge_y, CV_64F, 1, 0, 1);
  cv::convertScaleAbs(edge_y, abs_edge_y);
  cv::threshold(abs_edge_y,th_edge,6, 255, cv::THRESH_BINARY);

  /*canny
  cv::Mat detected_edges;
  int lowThreshold = 1;
  int ratio = 5;
  int kernel_size = 3;
  cv::blur( edge_frame, detected_edges, cv::Size(3,3) );
  cv::Canny( detected_edges, detected_edges, 5, 100 , kernel_size );
  th_edge =detected_edges;
  //th_edge = th_edge.mul((detected_edges));
  //

  //open
  //int morph_size = 1;
  //cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  //int kernelOpen[9] = {0, 1, 0, 0, 1, 0, 0, 1, 0};
  //cv::Mat element_open = cv::Mat(3, 3, CV_32F, kernelOpen);  
 // cv::Mat open_th_edge = th_edge;
  
    // For Erosion

  
  /* dilate
  int dilation_size = 0;
  cv::Mat element_dilate = getStructuringElement( cv::MORPH_ELLIPSE,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );
  dilate( th_edge, th_edge, element_dilate );

  cv::morphologyEx(th_edge, th_edge, cv::MORPH_OPEN,  open_kernel,cv::Point(1,1),1);

  */

  

  cv::Mat curr_edge = l_img.clone();

  
  cv::Mat th_edge_inv;
  cv::bitwise_not(th_edge, th_edge_inv);
  th_edge_inv = th_edge_inv/255.0;
  th_edge_inv.convertTo(th_edge_inv, CV_16U);

  th_edge.convertTo(th_edge, CV_16U, 1 / 255.0);  
  curr_edge = curr_edge.mul((th_edge));

  ground_mask2.convertTo(ground_mask2, CV_16U);  
  //curr_edge = curr_edge.mul((ground_mask2));


//surf
  cv::Mat ground_mask_inv;
  cv::Mat curr_surf = l_img.clone();
  cv::bitwise_not(ground_mask1, ground_mask_inv);
  ground_mask_inv = ground_mask_inv/255.0;
  ground_mask_inv.convertTo(ground_mask_inv, CV_16U);  

  //curr_surf = curr_surf.mul((ground_mask_inv)); // ground
  curr_surf = curr_surf.mul((th_edge_inv)); // surf and ground

  ////////////// without ground
  
  //curr_surf = curr_surf.mul((ground_mask2));
  //////////////
 
 
//ground segmentation
  cv::Mat curr_ground = l_img.clone();
  curr_ground = curr_ground - (curr_edge + curr_surf);

  

// point cloud reconstruction
  cv::Mat pc_surf,pc_edge;

  Eigen::Matrix<float,Dynamic,Dynamic> x_surf, y_surf, x_edge, y_edge;
  
  /*cv2eigen(curr_edge,x_edge);
  cv2eigen(curr_edge,y_edge);
  cv2eigen(curr_surf,x_surf);
  cv2eigen(curr_surf,y_surf);*/

  Eigen::Matrix<float,Dynamic,Dynamic> edge_sz;
  cv2eigen(curr_edge,edge_sz);  
  Eigen::Matrix<float,Dynamic,Dynamic> surf_sz;
  cv2eigen(curr_surf,surf_sz);

  Eigen::Matrix<float,Dynamic,Dynamic> ground_sz;
  cv2eigen(curr_ground,ground_sz);
  

  //pcl::PointCloud<pcl::PointXYZ> edge_cloud;
  PointCloud::Ptr edge_cloud (new PointCloud);
  edge_cloud->width = l_img.cols; 
  edge_cloud->height = l_img.rows;
  edge_cloud ->is_dense = false;
  edge_cloud->points.resize (edge_cloud->width * edge_cloud->height);

  //pcl::PointCloud<pcl::PointXYZ> surf_cloud;
  PointCloud::Ptr surf_cloud (new PointCloud);
  surf_cloud->width = l_img.cols; 
  surf_cloud->height = l_img.rows;
  surf_cloud ->is_dense = false;
  surf_cloud->points.resize (surf_cloud->width * surf_cloud->height);


    //pcl::PointCloud<pcl::PointXYZ> surf_cloud;
  PointCloud::Ptr ground_cloud (new PointCloud);
  ground_cloud->width = l_img.cols; 
  ground_cloud->height = l_img.rows;
  ground_cloud ->is_dense = false;
  ground_cloud->points.resize (ground_cloud->width * ground_cloud->height);

  int numpe = 0, numps =0 , numpg = 0;
  for (int i=0; i< l_img.rows; i+=1)
   {       
      for (int j=0; j<l_img.cols ; j+=1)
      {
        float ang= M_PI - ((2.0 * M_PI * j )/(l_img.cols));
        
        if(!(edge_sz(i,j)== 0 || edge_sz(i,j) > (pow(2,16)- (min_depth/max_depth) * pow(2,16)))){
        //if(!(edge_sz(i,j))== 0){ 
         // continue;
        //}
       // std::cout<<"I: "<<i<<endl;
       // std::cout<<"J: "<<j<<endl;

      //  std::cout<<"edge: "<<edge_sz(i,j)<<endl;
        float edge_data = (pow(2,16)-edge_sz(i,j))*(max_depth)/pow(2,16);                
        float edge_x_data = sqrt(pow(edge_data,2)- pow(z_range(i,j),2)) * cos(ang);
        float edge_y_data = sqrt(pow(edge_data,2)- pow(z_range(i,j),2)) * sin(ang);
        
        //edge_cloud->push_back(edge_x_data,edge_y_data,z_range(i,j);)
        edge_cloud->points[numpe].x = edge_x_data;
        edge_cloud->points[numpe].y = edge_y_data;
        edge_cloud->points[numpe].z = z_range(i,j);

        numpe++;

      }
      //if(!(surf_sz(i,j)== 0 || surf_sz(i,j) > (pow(2,16)- (min_depth/max_depth) * pow(2,16)))){
      if(!(surf_sz(i,j)== 0)){
        //  continue;
        //}

        float surf_data = (pow(2,16)-surf_sz(i,j))*(max_depth)/pow(2,16);                
        float surf_x_data = sqrt(pow(surf_data,2)- pow(z_range(i,j),2)) * cos(ang);
        float surf_y_data = sqrt(pow(surf_data,2)- pow(z_range(i,j),2)) * sin(ang);
        
        surf_cloud->points[numps].x = surf_x_data;
        surf_cloud->points[numps].y = surf_y_data;
        surf_cloud->points[numps].z = z_range(i,j);

        numps++;
      }


      if(!(ground_sz(i,j)== 0 || ground_sz(i,j) > (pow(2,16)- (min_depth/max_depth) * pow(2,16)))){
        //if(ground_sz(i,j)== 0){
         //continue;
        //}

        float ground_data = (pow(2,16)-ground_sz(i,j))*(max_depth)/pow(2,16);                
        float groundx_data = sqrt(pow(ground_data,2)- pow(z_range(i,j),2)) * cos(ang);
        float groundy_data = sqrt(pow(ground_data,2)- pow(z_range(i,j),2)) * sin(ang);
        
        ground_cloud->points[numpg].x = groundx_data;
        ground_cloud->points[numpg].y = groundy_data;
        ground_cloud->points[numpg].z = z_range(i,j);

        numpg++;
        }////// cierre del if
      }
   }  



  cv_bridge::CvImage out_msg;
  out_msg.header   = imgIn->header; // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
  out_msg.image    = img_frame_eq; // Your cv::Mat
  pub_img.publish(out_msg.toImageMsg()); 

  //cv_bridge::CvImage out_msg;
  //out_msg.header   = imgIn->header; // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::MONO16; // Or whatever
  out_msg.image    = curr_edge; // Your cv::Mat
  pub_edge.publish(out_msg.toImageMsg()); 

  //cv_bridge::CvImage out_msg;
  //out_msg.header   = imgIn->header; // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::MONO16; // Or whatever
  out_msg.image    = curr_surf; // Your cv::Mat
  pub_surf.publish(out_msg.toImageMsg()); 

  out_msg.encoding = sensor_msgs::image_encodings::MONO16; // Or whatever
  out_msg.image    = curr_ground; // Your cv::Mat
  pub_ground.publish(out_msg.toImageMsg()); 

  edge_cloud->header.frame_id = "base_link";
  surf_cloud->header.frame_id = "base_link";
  ground_cloud->header.frame_id = "base_link";
  //pcl_conversions::toPCL(ros::Time::now(), edge_cloud->header.stamp);
  pub_1.publish (edge_cloud);
  pub_2.publish (surf_cloud);
  pub_3.publish (ground_cloud);

  end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000.0;
                total_time+=time_temp;
                time_delay = total_time/total_frame;
                ROS_INFO("average process PC estimation time %f mS", time_delay);
                

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_pc_filter");
  ros::NodeHandle nh;  

  /// Load Parameters

  //nh.getParam("/max_depth", max_depth);
  //nh.getParam("/min_depth", min_depth);
  //nh.getParam("/imgTopic", imgTopic);

  //message_filters::Subscriber<Image> img_sub(nh, imgTopic, 1);

  //typedef sync_policies::ApproximateTime<PointCloud2, Image> MySyncPolicy;
  //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, img_sub);
  //sync.registerCallback(boost::bind(&callback, _1, _2));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/depth_image", 1, callback);
  pub_img = nh.advertise<sensor_msgs::Image>("/eq_image", 1);
  pub_edge = nh.advertise<sensor_msgs::Image>("/edge_image", 1);
  pub_surf = nh.advertise<sensor_msgs::Image>("/surf_image", 1);
  pub_ground = nh.advertise<sensor_msgs::Image>("/ground_image", 1);

  pub_1= nh.advertise<PointCloud> ("/pc_edge", 100);
  pub_2 = nh.advertise<PointCloud> ("/pc_surf", 100);
  pub_3 = nh.advertise<PointCloud> ("/pc_ground", 100);

  ros::spin();
  //return 0;
}

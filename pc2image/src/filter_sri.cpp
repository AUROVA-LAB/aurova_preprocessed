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

#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <limits>
#include <chrono> 


using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub_img;
ros::Publisher pub_edge;
ros::Publisher pub_surf;
ros::Publisher pub_ground;
ros::Publisher pub_1;
ros::Publisher pub_2;
ros::Publisher pub_3;

std::string depthImageTopic = "/depth_image/interpol";
std::string zImageTopic     = "/z_image/interpol";

int total_frame=0;
float time_delay  = 0;
double total_time =0;

void callback(const ImageConstPtr& imgIn_depth, const ImageConstPtr& imgIn_z)
{

  std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();

    cv_bridge::CvImagePtr cv_depth, cv_z;
        try
        {
          cv_depth = cv_bridge::toCvCopy(imgIn_depth, sensor_msgs::image_encodings::MONO16);
          cv_z     = cv_bridge::toCvCopy(imgIn_z    , sensor_msgs::image_encodings::MONO16);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

  cv::Mat img_r = cv_depth->image;  
  cv::Mat img_z = cv_z    ->image;  

  double r_max, r_min, z_max, z_min;  // max and min values of the images
  cv::minMaxLoc(img_r, &r_min, &r_max);
  cv::minMaxLoc(img_z, &z_min, &z_max);

  int cols = img_r.cols;
  int rows = img_r.rows;

  Eigen::Matrix<float,Dynamic,Dynamic> z_data_in,z_data;
  Eigen::Matrix<float,Dynamic,Dynamic> r_data_in,r_data;
  cv2eigen(img_z,z_data); 
  cv2eigen(img_r,r_data);

  //r_data = r_data_in*((r_max - r_min)/ pow(2,16)) + r_min; 
  //z_data = z_data_in*((z_max - z_min)/ pow(2,16)) + z_min; 
    

  /////// ///////////////////////////////equalize image ////////////////////////////////
  cv::Mat img_frame_eq;
  cv::Mat img_frame = img_r.clone();
  img_frame.convertTo(img_frame, CV_8UC1, 1 / 256.0);
  cv::equalizeHist(img_frame, img_frame_eq); 

  ////////////////////////////////////////ground filter ///////////////////////////////
  cv::Mat grad_x, abs_grad_x ,th_sobel;
  cv::Sobel(img_frame_eq, grad_x, CV_64F, 0, 1, 1);
  cv::convertScaleAbs(grad_x, abs_grad_x);
  cv::threshold(abs_grad_x,th_sobel,1, 255, cv::THRESH_BINARY_INV);

  cv::Mat ground_mask1, ground_mask2;
  int image_data[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  cv::Mat element = cv::Mat(3, 3, CV_32F, image_data);  
  cv::morphologyEx(th_sobel, ground_mask1,cv::MORPH_CLOSE, element,cv::Point(1,1),1);

  //open
  cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(ground_mask1, ground_mask1, cv::MORPH_CLOSE,  open_kernel,cv::Point(1,1),5);
  //

  // Edge 
  cv::Mat edge_y, abs_edge_y ,th_edge, edge_frame;
  ground_mask2 = ground_mask1/255.0;
  edge_frame = img_frame.mul((ground_mask2));
  cv::Sobel(edge_frame, edge_y, CV_64F, 1, 0, 1);
  cv::convertScaleAbs(edge_y, abs_edge_y);
  cv::threshold(abs_edge_y,th_edge,1, 255, cv::THRESH_BINARY);

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

  */
  
    // For Erosion

  
  /* dilate
  int dilation_size = 0;
  cv::Mat element_dilate = getStructuringElement( cv::MORPH_ELLIPSE,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );
  dilate( th_edge, th_edge, element_dilate );

  cv::morphologyEx(th_edge, th_edge, cv::MORPH_OPEN,  open_kernel,cv::Point(1,1),1);

  */  

  cv::Mat curr_edge = img_r.clone();
  
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
  cv::Mat curr_surf = img_r.clone();
  cv::bitwise_not(ground_mask1, ground_mask_inv);
  ground_mask_inv = ground_mask_inv/255.0;
  ground_mask_inv.convertTo(ground_mask_inv, CV_16U);  

  curr_surf = curr_surf.mul((ground_mask_inv)); // ground
  

  //curr_surf = curr_surf.mul((th_edge_inv)); // surf and ground

  ////////////// without ground
  
  //curr_surf = curr_surf.mul((ground_mask2));
  //////////////

 
//ground segmentation
  cv::Mat curr_ground = img_r.clone();
  curr_ground = curr_ground - (curr_edge + curr_surf);


// point cloud reconstruction
  cv::Mat pc_surf,pc_edge;

  Eigen::Matrix<float,Dynamic,Dynamic> x_surf, y_surf, x_edge, y_edge;
  
  //pcl::PointCloud<pcl::PointXYZ> edge_cloud;
  PointCloud::Ptr edge_cloud (new PointCloud);
  edge_cloud->width  = img_r.cols; 
  edge_cloud->height = img_r.rows;
  edge_cloud ->is_dense = false;
  edge_cloud->points.resize (edge_cloud->width * edge_cloud->height);

  //pcl::PointCloud<pcl::PointXYZ> surf_cloud;
  PointCloud::Ptr surf_cloud (new PointCloud);
  surf_cloud->width  = img_r.cols; 
  surf_cloud->height = img_r.rows;
  surf_cloud ->is_dense = false;
  surf_cloud->points.resize (surf_cloud->width * surf_cloud->height);


    //pcl::PointCloud<pcl::PointXYZ> surf_cloud;
  PointCloud::Ptr ground_cloud (new PointCloud);
  ground_cloud->width  = img_r.cols; 
  ground_cloud->height = img_r.rows;
  ground_cloud ->is_dense = false;
  ground_cloud->points.resize (ground_cloud->width * ground_cloud->height);

  Eigen::Matrix<float,Dynamic,Dynamic> edge_sz;
  cv2eigen(curr_edge,edge_sz);  
  Eigen::Matrix<float,Dynamic,Dynamic> surf_sz;
  cv2eigen(curr_surf,surf_sz);
  Eigen::Matrix<float,Dynamic,Dynamic> ground_sz;
  cv2eigen(curr_ground,ground_sz);

  int numpe = 0, numps =0 , numpg = 0;
  for (int i=0; i< img_r.rows; i+=1)
   {       
      for (int j=0; j<img_r.cols ; j+=1)
      {
        float ang= M_PI - ((2.0 * M_PI * j )/(img_r.cols));
        
        if(!(edge_sz(i,j)== 0 || edge_sz(i,j) > (pow(2,16)- (r_min/r_max) * pow(2,16)))){
        //if(!(edge_sz(i,j))== 0){ 
        // continue;
        //}
        // std::cout<<"I: "<<i<<endl;
        // std::cout<<"J: "<<j<<endl;

       //  std::cout<<"edge: "<<edge_sz(i,j)<<endl;
        //float edge_data = (pow(2,16)-edge_sz(i,j))*(r_max)/pow(2,16);  
        float edge_data  = (pow(2,16)-edge_sz(i,j))*((r_max - r_min)/ pow(2,16)) + r_min;  
        float z_position = (pow(2,16)-z_data(i,j)) *((z_max - z_min)/ pow(2,16)) + z_min;               
        float edge_x_data = sqrt(pow(edge_data,2)- pow(z_position,2)) * cos(ang);
        float edge_y_data = sqrt(pow(edge_data,2)- pow(z_position,2)) * sin(ang);
        
        //edge_cloud->push_back(edge_x_data,edge_y_data,z_data(i,j);)
        edge_cloud->points[numpe].x = edge_x_data;
        edge_cloud->points[numpe].y = edge_y_data;
        edge_cloud->points[numpe].z = z_position;

        numpe++;

      }
      else{
          curr_edge.at<ushort>(i, j) = 0;         
      }

      //if(!(surf_sz(i,j)== 0 || surf_sz(i,j) > (pow(2,16)- (min_depth/max_depth) * pow(2,16)))){
      if(!(surf_sz(i,j)== 0)){

        /*if(i < (l_img.rows/2)){   ///// Esto solo funciona cuando es con el velodyne
         //surf_sz(i,j) = 0; 
         curr_surf.at<ushort>(i, j) = 0;
         continue;
        }*/

        //  
        //}        

        float surf_data = (pow(2,16)-surf_sz(i,j))*(r_max)/pow(2,16);                
        float surf_x_data = sqrt(pow(surf_data,2)- pow(z_data(i,j),2)) * cos(ang);
        float surf_y_data = sqrt(pow(surf_data,2)- pow(z_data(i,j),2)) * sin(ang);
        
        surf_cloud->points[numps].x = surf_x_data;
        surf_cloud->points[numps].y = surf_y_data;
        surf_cloud->points[numps].z = z_data(i,j);

        numps++;
      }
      else{
          curr_surf.at<ushort>(i, j) = 0;
      }


      if(!(ground_sz(i,j)== 0 || ground_sz(i,j) > (pow(2,16)- (r_min/r_max) * pow(2,16)))){

        float ground_data = (pow(2,16)-ground_sz(i,j))*(r_max)/pow(2,16);                
        float groundx_data = sqrt(pow(ground_data,2)- pow(z_data(i,j),2)) * cos(ang);
        float groundy_data = sqrt(pow(ground_data,2)- pow(z_data(i,j),2)) * sin(ang);
        
        ground_cloud->points[numpg].x = groundx_data;
        ground_cloud->points[numpg].y = groundy_data;
        ground_cloud->points[numpg].z = z_data(i,j);

        numpg++;
        }
        else{
          curr_ground.at<ushort>(i, j) = 0;
      }////// cierre del if
      }
   }  


  /*PointCloud::Ptr cloud_filtered (new PointCloud);
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (surf_cloud);
  sor.setLeafSize (1.0, 1.0, 1.0);
  sor.filter (*cloud_filtered); */


  cv_bridge::CvImage out_msg;
  out_msg.header   = imgIn_depth->header; // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
  out_msg.image    = img_frame; // Your cv::Mat
  pub_img.publish(out_msg.toImageMsg()); 

  //cv_bridge::CvImage out_msg;
  //out_msg.header   = imgIn_depth->header; // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::MONO16; // Or whatever
  out_msg.image    = curr_edge; // Your cv::Mat
  pub_edge.publish(out_msg.toImageMsg()); 

  //cv_bridge::CvImage out_msg;
  //out_msg.header   = imgIn_depth->header; // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::MONO16; // Or whatever
  out_msg.image    = curr_surf; // Your cv::Mat
  pub_surf.publish(out_msg.toImageMsg()); 

  out_msg.encoding = sensor_msgs::image_encodings::MONO16; // Or whatever
  out_msg.image    = curr_ground; // Your cv::Mat
  pub_ground.publish(out_msg.toImageMsg()); 

  edge_cloud->header.frame_id = "velodyne";
  surf_cloud->header.frame_id = "velodyne";
  ground_cloud->header.frame_id = "velodyne";
  ros::Time time_st = imgIn_depth->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  edge_cloud->header.stamp    =  time_st.toNSec()/1e3;
  surf_cloud->header.stamp    =  time_st.toNSec()/1e3;
  ground_cloud->header.stamp  =  time_st.toNSec()/1e3;
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

  nh.getParam("/depthImg_Topic", depthImageTopic);
  nh.getParam("/zImg_Topic"    , zImageTopic    );

  message_filters::Subscriber<Image> range_sub (nh, depthImageTopic,  10);
  message_filters::Subscriber<Image> z_sub(nh, zImageTopic , 10);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), range_sub, z_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  pub_img = nh.advertise<Image>("/depth_img", 1);
  pub_edge = nh.advertise<Image>("/edge_image", 1);
  pub_surf = nh.advertise<Image>("/surf_image", 1);
  pub_ground = nh.advertise<Image>("/ground_image", 1);

  pub_1= nh.advertise<PointCloud> ("/pc_edge", 100);
  pub_2 = nh.advertise<PointCloud> ("/pc_ground", 100);  // dara la vuelta con la linea de abajo para cambiar solo ground
  pub_3 = nh.advertise<PointCloud> ("/pc_surf", 100);

  ros::spin();
  //return 0;
}

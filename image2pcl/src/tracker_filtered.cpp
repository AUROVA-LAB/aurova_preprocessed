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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <limits>
#include <chrono> 

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


//Publisher
ros::Publisher pc_filtered_pub; // publisher de la imagen de puntos filtrada
ros::Publisher goal_pub; // markers
PointCloud::Ptr pub_pointCloud (new PointCloud);

float bb_per = 0.8;  // bounding box reduction percentage 
float desv = 1; // desviacion en porcentaje
float bb_aug = 0.5; // bounding box augmentation percentage
float prev_goal_x=0, prev_goal_y=0;

// input topics 
std::string rangeTopic  = "/ouster/range_image";
std::string pcTopic     = "/ouster/points";
std::string objYoloTopic= "/yolov5/detections";
std::string filt_method= "median";

///////////////////////////////////////callback
int remap(int x, int limit){
  if(x<0) return limit+x;
  return x;
}

void callback(const ImageConstPtr& in_image, 
              const boost::shared_ptr<const detection_msgs::BoundingBoxes> &bb_data)
{

  pub_pointCloud->clear(); // clear data pointcloud
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
  detection_msgs::BoundingBoxes data = *bb_data;
  uint num_yolo_detection = data.bounding_boxes.size();

  //If there are 2 boxes, then the target is backwards, inj the limits of the image.
  int ymin=0, ymax=0, xmin=0, xmax=0;
  if (num_yolo_detection==2){
    ymin = data.bounding_boxes[0].ymin;
    ymax = data.bounding_boxes[0].ymax;
    if(data.bounding_boxes[0].xmin==0)
      xmax=data.bounding_boxes[0].xmax, xmin=data.bounding_boxes[1].xmin-img_range.cols;
    else
      xmax=data.bounding_boxes[1].xmax, xmin=data.bounding_boxes[0].xmin-img_range.cols;
  }
  else if (num_yolo_detection==1){
    ymin = data.bounding_boxes[0].ymin;
    ymax = data.bounding_boxes[0].ymax;
    xmin = data.bounding_boxes[0].xmin;
    xmax = data.bounding_boxes[0].xmax;
  }

  PointCloud::Ptr point_cloud (new PointCloud);
  PointCloud::Ptr cloud (new PointCloud);

  point_cloud->width = img_range.cols; 
  point_cloud->height = img_range.rows;
  point_cloud->is_dense = false;
  point_cloud->points.resize (point_cloud->width * point_cloud->height);
  uint num_pix = 0;

  float median = 0, goal_x=0, goal_y=0;
  if(num_yolo_detection>0){
    // Calculate the median depth

    float depth_ave = 0;   //  average distance of object
    uint cont_pix=0;        // number of pixels 

    uint start_x = (1-bb_per/2.0) * xmin + (bb_per/2.0 * xmax);
    uint end_x   = (1-bb_per/2.0) * xmax + (bb_per/2.0 * xmin);
    uint start_y = (1-bb_per/2.0) * ymin + (bb_per/2.0 * ymax);
    uint end_y   = (1-bb_per/2.0) * ymax + (bb_per/2.0 * ymin);

    // optimizar recortando matrix de direccion a hasta b y luego sacar media
    std::vector<float> vec_std_depth; // vector para medianas

    for (int iy = start_y;iy<end_y; iy++)
      for (int ix = start_x;ix<end_x; ix++)
          if(data_metrics(iy,remap(ix,img_range.cols))!=0){
            depth_ave += data_metrics(iy,remap(ix,img_range.cols));
            cont_pix++;
            vec_std_depth.push_back(data_metrics(iy,remap(ix,img_range.cols)));
          }
      // condicion para que no de valores infinitos     
    if(depth_ave == 0 && cont_pix==0){
      cont_pix = 1;
      vec_std_depth.push_back(0);
    }

    // punto medio 
    int Ox = (xmax+xmin)/2;
    int Oy = (ymax+ymin)/2;
    float p_med = data_metrics(Oy,remap(Ox,img_range.cols));


    /////// calculo de la mediana ////////////////////////////////////////
    int n = sizeof(vec_std_depth) / sizeof(vec_std_depth[0]);  
    sort(vec_std_depth.begin(), vec_std_depth.begin() + n, greater<int>());
    int tam = vec_std_depth.size();
    
    if (tam % 2 == 0) {  
        median = (vec_std_depth[((tam)/2) -1] + vec_std_depth[(tam)/2])/2.0; 
    }      
    else { 
        if(tam==1)
        median = vec_std_depth[tam];
      else
        median = vec_std_depth[tam/2];
    }         
    vec_std_depth.clear();

    //Calculate goal as the  center of the bounding box but using the median depth
    float ang_h = 22.5 - (45.0/128.0)*Oy;
    ang_h = ang_h*M_PI/180.0;
    float ang_w = 184.0 - (360.0/2048.0)*remap(Ox,img_range.cols);
    ang_w = ang_w*M_PI/180.0;
    float z = median * sin(ang_h);
    goal_y = sqrt(pow(median,2)-pow(z,2))*sin(ang_w);
    goal_x = sqrt(pow(median,2)-pow(z,2))*cos(ang_w);
    
    ///////////////////////////////////////////////////////
    depth_ave = depth_ave/cont_pix;
    // std::cout<<"Depth average: "<<depth_ave << " median: "<<median << "Punto medio: "<<p_med<< std::endl;  
    // std::cout<<"BB_per: "<<bb_per<<" filter_dev: "<<desv << std::endl;
    // std::cout<<"Goal x: "<<goal_x<<" Y: "<<goal_y<<" Z: "<<z<<std::endl;
  }
  else{
    goal_x=0.9*prev_goal_x; goal_y=0.9*prev_goal_y;
  }

  for (uint iy = 0;iy<img_range.rows; iy++){
    for (uint ix = 0;ix<img_range.cols; ix++){        

      // recosntruccion de la nube de puntos
      if (data_metrics(iy,ix)==0)
        continue;

      //Remove points of the target.
      //filtrado por desviacion de puntos con el valor de profundidad
      float aug_h=(ymax-ymin)*bb_aug/2, aug_w=(xmax-xmin)*bb_aug/2;
      if (!(iy>ymin-aug_h && iy<ymax+aug_h && ix<xmax+aug_w && (ix>xmin-aug_w || (num_yolo_detection==2 && ix>remap(xmin-aug_w,img_range.cols))) &&
        (data_metrics(iy,ix)> median-desv && data_metrics(iy,ix)< median+desv))){          

        float ang_h = 22.5 - (45.0/128.0)*iy;
        ang_h = ang_h*M_PI/180.0;
        float ang_w = 184.0 - (360.0/2048.0)*ix;
        ang_w = ang_w*M_PI/180.0;

        float z = data_metrics(iy,ix) * sin(ang_h);
        float y = sqrt(pow(data_metrics(iy,ix),2)-pow(z,2))*sin(ang_w);
        float x = sqrt(pow(data_metrics(iy,ix),2)-pow(z,2))*cos(ang_w);
        //asignacion de valores maximo y minimos

        point_cloud->points[num_pix].x = x;
        point_cloud->points[num_pix].y = y;
        point_cloud->points[num_pix].z = z;
        cloud->push_back(point_cloud->points[num_pix]); 
        num_pix++;   
      } 
    }
  }

  // Remuestrear la nube de puntos para acelerar el procesamiento
  /*pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.1f, 0.1f, 0.1f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  vg.filter(*cloud_filtered);
  
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  PointCloud::Ptr cloud_in (new PointCloud);
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.2);
  sor.filter (*cloud_in);*/
  
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *pub_pointCloud, indices);

  pub_pointCloud->is_dense = true;
  pub_pointCloud->width = (int) pub_pointCloud->points.size();
  pub_pointCloud->height = 1;
  pub_pointCloud->header.frame_id = "os_sensor";
  ros::Time time_st = bb_data->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  pub_pointCloud->header.stamp = time_st.toNSec()/1e3;
  pc_filtered_pub.publish (pub_pointCloud);
  
  geometry_msgs::PoseWithCovarianceStamped goal_msg;
  goal_msg.header=bb_data->header; goal_msg.header.frame_id= "os_sensor";
  goal_msg.pose.pose.position.x=goal_x; goal_msg.pose.pose.position.y=goal_y;
  goal_pub.publish(goal_msg);
  prev_goal_x=goal_x; prev_goal_y=goal_y;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "pontCloudOntImage");
  ros::NodeHandle nh("~");  
  std::cout<<"Nodo inicializado: "<<std::endl;
  
  /// Load Parameters

  nh.getParam("/pcTopic", pcTopic);
  nh.getParam("/range_img", rangeTopic);
  nh.getParam("/detection_BoundingBoxes", objYoloTopic);
  nh.getParam("/filtering_method", filt_method);
  nh.getParam("/bounding_box_percet_reduction", bb_per);
  nh.getParam("/filtering_desviation", desv);
  nh.getParam("/bounding_box_percet_augmentation", bb_aug);

  //Get the node rate.
  double r;
  nh.getParam("rate", r);
  ros::Rate rate(r);
  
  message_filters::Subscriber<Image>range_sub (nh, rangeTopic,  10);
  message_filters::Subscriber<detection_msgs::BoundingBoxes> yoloBB_sub(nh, objYoloTopic , 10);

  typedef sync_policies::ApproximateTime<Image, detection_msgs::BoundingBoxes> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), range_sub, yoloBB_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  pc_filtered_pub = nh.advertise<PointCloud> ("/ouster_filtered", 1);  
  goal_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( "/target", 1 );

  while (ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
  
}

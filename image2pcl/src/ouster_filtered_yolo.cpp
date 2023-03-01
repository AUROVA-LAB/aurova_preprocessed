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
ros::Publisher vis_pub; // markers
PointCloud::Ptr pub_pointCloud (new PointCloud);

float bb_per = 0.8;  // bounding box reduction percentage 
float desv = 0.1; // desviacion en porcentaje

// input topics 
std::string rangeTopic  = "/ouster/range_image";
std::string pcTopic     = "/ouster/points";
std::string objYoloTopic= "/yolov5/detections";
std::string filt_method= "median";

///////////////////////////////////////callback


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

  PointCloud::Ptr point_cloud (new PointCloud);
  PointCloud::Ptr cloud (new PointCloud);

  point_cloud->width = img_range.cols; 
  point_cloud->height = img_range.rows;
  point_cloud->is_dense = false;
  point_cloud->points.resize (point_cloud->width * point_cloud->height);
  uint num_pix = 0;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "os_sensor";
  marker.header.stamp = bb_data->header.stamp;
  marker.ns = "person";
  marker.action = visualization_msgs::Marker::DELETEALL;

  for(uint i=0 ;i<num_yolo_detection; i++)
	{
    uint xmin = data.bounding_boxes[i].xmin;
    uint ymin = data.bounding_boxes[i].ymin;
    uint xmax = data.bounding_boxes[i].xmax;
    uint ymax = data.bounding_boxes[i].ymax;

    float depth_ave = 0;   //  average distance of object
    uint cont_pix=0;        // number of pixels 
  
    uint start_x = (1-bb_per/2.0) * xmin + (bb_per/2.0 * xmax);
    uint end_x   = (1-bb_per/2.0) * xmax + (bb_per/2.0 * xmin);
    uint start_y = (1-bb_per/2.0) * ymin + (bb_per/2.0 * ymax);
    uint end_y   = (1-bb_per/2.0) * ymax + (bb_per/2.0 * ymin);

    // optimizar recortando matrix de direccion a hasta b y luego sacar media
    std::vector<float> vec_std_depth; // vector para medianas

    for (uint iy = start_y;iy<end_y; iy++)
      for (uint ix = start_x;ix<end_x; ix++)
          if(data_metrics(iy,ix)!=0){
            depth_ave += data_metrics(iy,ix);
            cont_pix++;
            vec_std_depth.push_back(data_metrics(iy,ix));
          }
     // condicion para que no de valores infinitos     
    if(depth_ave == 0 && cont_pix==0){
      cont_pix = 1;
      vec_std_depth.push_back(0);
    }

    // punto medio 
    int Ox = (xmax+xmin)/2;
    int Oy = (ymax+ymin)/2;
    float p_med = data_metrics(Oy,Ox);


  /////// calculo de la mediana ////////////////////////////////////////
    int n = sizeof(vec_std_depth) / sizeof(vec_std_depth[0]);  
    sort(vec_std_depth.begin(), vec_std_depth.begin() + n, greater<int>());
    int tam = vec_std_depth.size();
    float median = 0;
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
    ///////////////////////////////////////////////////////
    depth_ave = depth_ave/cont_pix;
    std::cout<<"Person: "<<i<<" average: "<<depth_ave << " median: "<<median << "Punto medio: "<<p_med<< std::endl;  
    std::cout<<"BB_per: "<<bb_per<<" filter_dev: "<<desv << std::endl; 

    for (uint iy = ymin;iy<ymax; iy++){
      for (uint ix = xmin;ix<xmax; ix++){        

          // recosntruccion de la nube de puntos
          if (data_metrics(iy,ix)==0)
            continue;

          if ((data_metrics(iy,ix)> -median*(1+desv) && data_metrics(iy,ix)< median*(1+desv))){  // filtrado por desviacion de puntos con el valor de profundidad        

          float ang_h = 22.5 - (45.0/128.0)*iy;
          ang_h = ang_h*M_PI/180.0;
          float ang_w = 184.0 - (360.0/2048.0)*ix;
          ang_w = ang_w*M_PI/180.0;

          float z = data_metrics(iy,ix) * sin(ang_h);
          if (z<-1.0)
            continue;
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
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_filtered);
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    PointCloud::Ptr cloud_in (new PointCloud);
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (50);
    sor.setStddevMulThresh (0.2);
    sor.filter (*cloud_in);
   
    PointCloud::Ptr cloud_out (new PointCloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, indices);

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*cloud_in, minPt, maxPt);
    cloud->clear();

    //pub_pointCloud->push_back(cloud_out);

    for (int j = 0; j < (int) cloud_out->points.size(); j++)
      {
        //  double distance = sqrt(cloud_in->points[j].x * cloud_in->points[j].x + cloud_in->points[j].y * cloud_in->points[j].y);
        //  if(distance<minlen || distance>maxlen)
        //      continue;
           
        pub_pointCloud->push_back(cloud_out->points[j]);  
        
      }

    double x_center = (maxPt.x+minPt.x)/2.0 ;
    double y_center = (maxPt.y+minPt.y)/2.0 ;
    double z_center = (maxPt.z+minPt.z)/2.0 ;
      // plot bounding box of personin RVIZ
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_center;
    marker.pose.position.y = y_center;
    marker.pose.position.z = z_center;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.25; 
    marker.color.r = (i*50.0)/255.0;
    marker.color.g = 1.0;
    marker.color.b = (i*50.0)/255.0;
    vis_pub.publish( marker ); 
  }

  pub_pointCloud->is_dense = true;
  pub_pointCloud->width = (int) pub_pointCloud->points.size();
  pub_pointCloud->height = 1;
  pub_pointCloud->header.frame_id = "/os_sensor";
  ros::Time time_st = bb_data->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  pub_pointCloud->header.stamp = time_st.toNSec()/1e3;
  pc_filtered_pub.publish (pub_pointCloud);
  

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "pontCloudOntImage");
  ros::NodeHandle nh;  
  std::cout<<"Nodo inicializado: "<<std::endl;
  
  /// Load Parameters

  nh.getParam("/pcTopic", pcTopic);
  nh.getParam("/range_img", rangeTopic);
  nh.getParam("/detection_BoundingBoxes", objYoloTopic);
  nh.getParam("/filtering_method", filt_method);
  nh.getParam("/bounding_box_percet_reduction", bb_per);
  nh.getParam("/filtering_desviation", desv);

  message_filters::Subscriber<Image>range_sub (nh, rangeTopic,  10);
  message_filters::Subscriber<detection_msgs::BoundingBoxes> yoloBB_sub(nh, objYoloTopic , 10);

  typedef sync_policies::ApproximateTime<Image, detection_msgs::BoundingBoxes> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), range_sub, yoloBB_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  pc_filtered_pub = nh.advertise<PointCloud> ("/ouster_filtered", 1);  
  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );

  ros::spin();
}

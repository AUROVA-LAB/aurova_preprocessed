#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Dense>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
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
#include <pcl/io/pcd_io.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>
#include <math.h>
#include <limits>
#include <chrono> 


using namespace Eigen;
using namespace sensor_msgs;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//Publisher
ros::Publisher pc_filtered_pub; 
ros::Publisher pc_limits_pub; 
ros::Publisher pc_obstXY_pub; 

// input topics 
std::string pcTopic     = "/ouster/points";

double smNorm = 0.1;
double laNorm = 1.0;
double th     = 0.5;
double vx_cloud = 0.2;

// initial parameters
float maxlen =100; // radio maximo de nube de puntos
float minlen = 0.0; // radio minimo de nube de puntos
float max_z = 0.5;  // todos los puntos que tengan sean mayores a esta altura son eliminados
float scaleRadio = 0.8; // escala del radio original para filtarr a la salida (debe ser menor a 1)
float th_meanK = 0.2;
float vecinos = 10;


//// Spherical image pcl
boost::shared_ptr<pcl::RangeImageSpherical> rngSpheric;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

typedef std::chrono::high_resolution_clock Clock;


///////////////////////////////////////callback


void callback(const PointCloud::ConstPtr& msg_pointCloud)
{
  if (msg_pointCloud == NULL) return;
  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud (new PointCloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*msg_pointCloud, *cloud_in, indices);

  for (int i = 0; i < (int) cloud_in->points.size(); i++)
  {
      double distance = sqrt(pow(cloud_in->points[i].x,2)+pow(cloud_in->points[i].y,2)+pow(cloud_in->points[i].z,2));
      if(distance<minlen || distance>maxlen || cloud_in->points[i].z> max_z)
        continue;
     cloud->push_back(cloud_in->points[i]);          
  }

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(vx_cloud, vx_cloud, vx_cloud);
  vg.filter(*cloud);

  // crea KDTree para datos no organizados
   pcl::search::Search<pcl::PointXYZ>::Ptr tree;
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  }
   tree->setInputCloud (cloud);

    // Calcular las normales
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
  
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);
  ne.setRadiusSearch (smNorm);// Radio de búsqueda para vecinos cercanos
  ne.compute (*normals_small_scale);
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);
  ne.setRadiusSearch (laNorm);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
  copyPointCloud (*cloud, *doncloud);

  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

   if (!don.initCompute ())
   {
     std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
     std::cerr << "Error:en deteccion de obstaculos " << std::endl;
     exit (EXIT_FAILURE);
   }
  // Compute DoN
  don.computeFeature (*doncloud);

  // Build the condition for filtering
  pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond ( new pcl::ConditionOr<pcl::PointNormal> ()  );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
                               new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, th))
                             );

  // Build the filter
  pcl::ConditionalRemoval<pcl::PointNormal> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (doncloud);

  // Apply filter
  pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);
  condrem.filter (*doncloud_filtered);
  doncloud = doncloud_filtered;

  // apply outlier filter

  pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
  sor.setInputCloud (doncloud);
  sor.setMeanK (vecinos);
  sor.setStddevMulThresh (th_meanK);
  sor.filter (*doncloud);

  // pointcloud pcl_obstacle , projection XY and free_obstacle  
  pcl::PointCloud<pcl::PointNormal>::Ptr pcl_obstacle (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_obstXY (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr limits_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl_obstacle =doncloud;

  pcl_obstXY->is_dense = false;
  pcl_obstXY->width = doncloud->width;
  pcl_obstXY->height = doncloud->height;
  pcl_obstXY->points.resize (doncloud->width * doncloud->height);

  limits_cloud->is_dense = false;
  limits_cloud->width = doncloud->width;
  limits_cloud->height = doncloud->height;
  limits_cloud->points.resize (doncloud->width * doncloud->height);

  for (int i = 0; i < (int) pcl_obstacle->points.size(); i++)
  {
    pcl_obstXY->points[i].x= pcl_obstacle->points[i].x;     
    pcl_obstXY->points[i].y= pcl_obstacle->points[i].y; 
    pcl_obstXY->points[i].z= 0.0;   
    
    // ring generator
    float ang =   M_PI-(i*2.0*M_PI)/pcl_obstacle->points.size();
    limits_cloud->points[i].x = maxlen*cos(ang);
    limits_cloud->points[i].y = maxlen*sin(ang);
    limits_cloud->points[i].z = 0.0;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_combined (new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_combined = *pcl_obstXY + *limits_cloud;  
 
  // pointcloud pcl_obstacle projection XY and free_obstacle  
  rngSpheric->pcl::RangeImage::createFromPointCloud(*cloud_combined, pcl::deg2rad(0.5), pcl::deg2rad(0.5),
                                       pcl::deg2rad(360.0), pcl::deg2rad(180.0),
                                       Eigen::Affine3f::Identity(), coordinate_frame, 0.0f, 0.0f, 0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr limits_cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_obstXY_out (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_var (new pcl::PointCloud<pcl::PointXYZ>);
  point_var->width = 1;
  point_var->height = 1;
  point_var->is_dense = false;
  point_var->points.resize (point_var->width * point_var->height);

  int cols = rngSpheric->width;
  int rows = rngSpheric->height;
  float xx=0, yy=0 ,r = maxlen;

  for (int i=0; i< int(cols); i++)    {
    float minRange = maxlen;
    bool obstacle = false;
    for (int j=0; j<rows ; j++)
      {  
        xx = rngSpheric->getPoint(i, j).x;
        yy = rngSpheric->getPoint(i, j).y; 
        r =  rngSpheric->getPoint(i, j).range;
        if (r == -INFINITY || r>=maxlen*scaleRadio)
          continue;          
        if (r < minRange ) {
          minRange = r; 
          obstacle = true;          
        }        
      }
    point_var->points[0].x = xx;
    point_var->points[0].y = yy;
    point_var->points[0].z = -1.1;
    if (obstacle) pcl_obstXY_out->push_back(point_var->points[0]);
    else        limits_cloud_out->push_back(point_var->points[0]);

  }
 
  // obstacles point cloud
  pcl_obstacle->is_dense = false;
  pcl_obstacle->width = pcl_obstacle->width;
  pcl_obstacle->height = pcl_obstacle->height;
  pcl_obstacle->points.resize (pcl_obstacle->width * pcl_obstacle->height);
  pcl_obstacle->header.frame_id = "os_sensor";
  pcl_obstacle->header.stamp = msg_pointCloud->header.stamp;
  pc_filtered_pub.publish (pcl_obstacle);

  // obstacles point cloud projection in XY plane  
  pcl_obstXY_out->is_dense = false;
  pcl_obstXY_out->width = pcl_obstXY_out->width;
  pcl_obstXY_out->height = pcl_obstXY_out->height;
  pcl_obstXY_out->points.resize (pcl_obstXY_out->width * pcl_obstXY_out->height);
  pcl_obstXY_out->header.frame_id = "os_sensor";
  pcl_obstXY_out->header.stamp = msg_pointCloud->header.stamp;
  pc_obstXY_pub.publish (pcl_obstXY_out);

  // limits cloud
  limits_cloud_out->is_dense = false;
  limits_cloud_out->width = limits_cloud_out->width;
  limits_cloud_out->height = limits_cloud_out->height;
  limits_cloud_out->points.resize (limits_cloud_out->width * limits_cloud_out->height);
  limits_cloud_out->header.frame_id = "os_sensor";
  limits_cloud_out->header.stamp = msg_pointCloud->header.stamp;
  pc_limits_pub.publish (limits_cloud_out);

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "LidarObstacle");
  ros::NodeHandle nh;  
  std::cout<<"Nodo obstaculos inicializado: "<<std::endl;
  
  /// Load Parameters

  nh.getParam("/pcTopic", pcTopic);
  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/maxZ_filter", max_z);
  nh.getParam("/smNorm", smNorm);
  nh.getParam("/laNorm", laNorm);
  nh.getParam("/threshold", th);
  nh.getParam("/voxel", vx_cloud);
  nh.getParam("/scaleRadio", scaleRadio);
  nh.getParam("/vecinos", vecinos);
  nh.getParam("/th_meanK", th_meanK);
  
  rngSpheric = boost::shared_ptr<pcl::RangeImageSpherical>(new pcl::RangeImageSpherical);
  
  ros::Subscriber sub = nh.subscribe<PointCloud>(pcTopic, 1, callback);  
  pc_filtered_pub = nh.advertise<PointCloud> ("/ouster_obstacles", 1);
  pc_obstXY_pub   = nh.advertise<PointCloud> ("/ground_obstacles", 1);
  pc_limits_pub   = nh.advertise<PointCloud> ("/ground_limits", 1);

  ros::spin();
}

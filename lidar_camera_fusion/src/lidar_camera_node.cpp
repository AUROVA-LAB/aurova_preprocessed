#include <ros/ros.h>
#include <sensor_msgs/Image.h>
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
#include <opencv2/core/core.hpp>
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
#include <armadillo>

#include <chrono> 

typedef std::chrono::high_resolution_clock Clock;


using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

//Publisher
ros::Publisher pcOnimg_pub;
ros::Publisher pc_pub;




float maxlen =100.0;       //maxima distancia del lidar
float minlen = 0.01;     //minima distancia del lidar
float max_FOV = 3.0;    // en radianes angulo maximo de vista de la camara
float min_FOV = 0.4;    // en radianes angulo minimo de vista de la camara

/// parametros para convertir nube de puntos en imagen
float angular_resolution_x =0.5f;
float angular_resolution_y = 2.1f;
float max_angle_width= 360.0f;
float max_angle_height = 180.0f;
float z_max = 100.0f;
float z_min = 100.0f;

float max_depth =100.0;
float min_depth = 8.0;

// topics a suscribirse del nodo
std::string imgTopic = "/camera/color/image_raw";
std::string pcTopic = "/velodyne_points";

// matrices de calibracion entre la camara y el lidar

Eigen::MatrixXf Tlc(3,1); // translation matrix lidar-camera
Eigen::MatrixXf Rlc(3,3); // rotation matrix lidar-camera
Eigen::MatrixXf Mc(3,4);  // camera calibration matrix

// range image parametros
boost::shared_ptr<pcl::RangeImageSpherical> rangeImage;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;


//// Segmantion plane 


///////////////////////funciones FFT
/*void fftshift(const cv::Mat &input_img, cv::Mat &output_img)
{
	output_img = input_img.clone();
	int cx = output_img.cols / 2;
	int cy = output_img.rows / 2;
	cv::Mat q1(output_img, cv::Rect(0, 0, cx, cy));
	cv::Mat q2(output_img, cv::Rect(cx, 0, cx, cy));
	cv::Mat q3(output_img, cv::Rect(0, cy, cx, cy));
	cv::Mat q4(output_img, cv::Rect(cx, cy, cx, cy));

	cv::Mat temp;
	q1.copyTo(temp);
	q4.copyTo(q1);
	temp.copyTo(q4);
	q2.copyTo(temp);
	q3.copyTo(q2);
	temp.copyTo(q3);
}


void calculateDFT(cv::Mat &scr, cv::Mat &dst)
{
	// define mat consists of two mat, one for real values and the other for complex values
	cv::Mat planes[] = { scr, cv::Mat::zeros(scr.size(), CV_32F) };
	cv::Mat complexImg;
	cv::merge(planes, 2, complexImg);

	cv::dft(complexImg, complexImg);
	dst = complexImg;
}

cv::Mat construct_H(cv::Mat &scr, cv::String type, float D0)
{
	cv::Mat H(scr.size(), CV_32F, cv::Scalar(1));
	float D = 0;
	if (type == "Ideal")
	{
		for (int u = 0; u < H.rows; u++)
		{
			for (int  v = 0; v < H.cols; v++)
			{
				D = sqrt((u - scr.rows / 2)*(u - scr.rows / 2) + (v - scr.cols / 2)*(v - scr.cols / 2));
				if (D > D0)
				{
					H.at<float>(u, v) = 0;
				}
			}
		}
		return H;
	}
	else if (type == "Gaussian")
	{
		for (int  u = 0; u < H.rows; u++)
		{
			for (int v = 0; v < H.cols; v++)
			{
				D = sqrt((u - scr.rows / 2)*(u - scr.rows / 2) + (v - scr.cols / 2)*(v - scr.cols / 2));
				H.at<float>(u, v) = exp(-D*D / (2 * D0*D0));
			}
		}
		return H;
	}
  else if (type == "lineal")
	{
		for (int  u = 0; u < H.rows; u++)
		{
			for (int v = (H.cols/2)-D0; v < (H.cols/2)+D0; v++)
			{
				H.at<float>(u, v) = 0;
			}
		}
		return H;
	}
}


void filtering(cv::Mat &scr, cv::Mat &dst, cv::Mat &H)
{
	fftshift(H, H);
	cv::Mat planesH[] = { cv::Mat_<float>(H.clone()), cv::Mat_<float>(H.clone()) };

	cv::Mat planes_dft[] = { scr, cv::Mat::zeros(scr.size(), CV_32F) };
	cv::split(scr, planes_dft);

	cv::Mat planes_out[] = { cv::Mat::zeros(scr.size(), CV_32F), cv::Mat::zeros(scr.size(), CV_32F) };
	planes_out[0] = planesH[0].mul(planes_dft[0]);
	planes_out[1] = planesH[1].mul(planes_dft[1]);

	cv::merge(planes_out, 2, dst);

}*/



///////////////////////////////////////callback



void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& in_pc2 , const ImageConstPtr& in_image)
{
  //ros::Rate rate(10); 
  auto t1 = Clock::now();

    cv_bridge::CvImagePtr cv_ptr , color_pcl;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::BGR8);
          color_pcl = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

  //Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*in_pc2,pcl_pc2);
  PointCloud::Ptr msg_pointCloud(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*msg_pointCloud);
  ///

  ////// filter point cloud 
  if (msg_pointCloud == NULL) return;

  PointCloud::Ptr cloud_in (new PointCloud);
  //PointCloud::Ptr cloud_filter (new PointCloud);
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

     //filtrado de la nube
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud_filter);
  sor.setMeanK (50.0);
  sor.setStddevMulThresh (0.1);
  sor.filter (*cloud_out);*/



  //                                                  point cloud to image 

  //============================================================================================================
  //============================================================================================================

  // range image    

                 // ,0  ,0     ,0      ,1;

  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  rangeImage->pcl::RangeImage::createFromPointCloud(*cloud_out, pcl::deg2rad(angular_resolution_x), pcl::deg2rad(angular_resolution_y),
                                       pcl::deg2rad(max_angle_width), pcl::deg2rad(max_angle_height),
                                       sensorPose, coordinate_frame, 0.0f, 0.0f, 0);

  

  int cols_img = rangeImage->width;
  int rows_img = rangeImage->height;


  arma::mat Z;  // interpolation de la imagen
  arma::mat Zz; // interpolation de las alturas de la imagen

  Z.zeros(rows_img,cols_img);         // rango
  Zz.zeros(rows_img,cols_img);        // altura

  Eigen::MatrixXf ZZei (rows_img,cols_img);
 
  for (int i=0; i< cols_img; ++i)
      for (int j=0; j<rows_img ; ++j)
      {
        float r =  rangeImage->getPoint(i, j).range;     
        float zz = rangeImage->getPoint(i, j).z; 

       
        Eigen::Vector3f tmp_point;
        rangeImage->calculate3DPoint (float(i), float(j), r, tmp_point);
        //std::cout<<tmp_point[0]<<" "<<tmp_point[1]<<" "<<tmp_point[2]<<std::endl;

        //float zz = tmp_point[2]; 

        if(std::isinf(r) || r<minlen || r>maxlen || std::isnan(zz)){
            continue;
        }             
        Z.at(j,i) = r;   
        Zz.at(j,i) = zz;
        ZZei(j,i)=zz;
       
      }

  ////////////////////////////////////////////// interpolation
  //============================================================================================================
  
  arma::vec X = arma::regspace(1, Z.n_cols);  // X = horizontal spacing
  arma::vec Y = arma::regspace(1, Z.n_rows);  // Y = vertical spacing 

  float interpol_value = 20.0;

  arma::vec XI = arma:: regspace(X.min(), 1.0, X.max()); // magnify by approx 2
  arma::vec YI = arma::regspace(Y.min(), 1.0/interpol_value, Y.max()); // // Y solo para poner el suelo

  //arma::vec YI = arma::regspace(Y.min(), 1.0/interpol_value, Y.max()); // // Y solo para poner el suelo
  

  arma::mat ZI_near;  
  arma::mat ZI;
  arma::mat ZzI;


 // arma::interp2(X, Y, Z, XI, YI, ZI_near,"nearest"); 
  arma::interp2(X, Y, Z, XI, YI, ZI,"lineal");  
  arma::interp2(X, Y, Zz, XI, YI, ZzI,"lineal");  





///////////////////////////////// filtrado por imagen

//===========================================inicio==========================================================
/*
  cv::Mat interdephtImage_near =  cv::Mat::zeros(ZI_near.n_rows, ZI_near.n_cols, cv_bridge::getCvType("mono16"));
  cv::Mat interdephtImage_lin =  cv::Mat::zeros(ZI.n_rows, ZI.n_cols, cv_bridge::getCvType("mono16"));
  cv::Mat interdephtImage_ZZ =  cv::Mat::zeros(ZzI.n_rows, ZzI.n_cols,cv_bridge::getCvType("mono16"));

  cv::Mat interdephtImage_out =  cv::Mat::zeros(ZI_near.n_rows, ZI_near.n_cols, cv_bridge::getCvType("mono16"));

  for (int i=0; i< ZI.n_cols; i++)
      for (int j=0; j<ZI.n_rows ; j++)
      {
        interdephtImage_near.at<ushort>(j, i)     = 1- ZI_near(j,i)*pow(2,16)/100.0;   
        interdephtImage_lin.at<ushort>(j, i) = 1- ZI(j,i)*pow(2,16)/100.0; 
        interdephtImage_ZZ.at<ushort>(j, i) = 1- ZzI(j,i)*pow(2,16)/100.0;        
        }



  /////////////////equalizacion de imagen  

  cv::Mat img_frame_eq;
  cv::Mat img_frame = interdephtImage_near.clone();
  img_frame.convertTo(img_frame, CV_8UC1, 1 / 256.0);
  cv::equalizeHist(img_frame, img_frame_eq); 

  ////////////////////////////////////////



  /////////////////////// Fourier con opencv////////////////////////////
  
  cv::Mat imgIn = img_frame.clone();
	imgIn.convertTo(imgIn, CV_32F);

	// DFT
	cv::Mat DFT_image;
	calculateDFT(imgIn, DFT_image);

	// construct H
	cv::Mat H;
	H = construct_H(imgIn, "lineal", 1);

	// filtering
	cv::Mat complexIH;
	filtering(DFT_image, complexIH, H);

	// IDFT
	cv::Mat imgOut;
	dft(complexIH, imgOut, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);

  cv::normalize(imgOut,imgOut,0,1,cv::NORM_MINMAX);

  ///////////////////////////////////////////////////////////////////

   
  for (int i=0; i< interdephtImage_out.cols; i++)
      for (int j=0; j<interdephtImage_out.rows; j++)
      {    

        unsigned int data = imgOut.at<float>(j,i) * (pow(2,16));
          
        //if(data > (45000-5000) && data < (45000+5000) ) {// elimino los valores del filtradod e fourier
       
       interdephtImage_out.at<ushort>(j, i) = data;  
         
       // }
        
        

      }
  
  ////////////// bordes
  cv::Mat edge_y, abs_edge_y ,th_edge, edge_frame; 
  edge_frame =  img_frame.clone();


  cv::Sobel(edge_frame, edge_y, CV_64F, 0, 1, 1);
  cv::convertScaleAbs(edge_y, abs_edge_y);
  cv::threshold(abs_edge_y,th_edge,0, 255, cv::THRESH_BINARY);


  cv::Mat curr_edge = interdephtImage_near.clone();
  
  cv::Mat th_edge_inv;
  cv::bitwise_not(th_edge, th_edge_inv);
  th_edge_inv = th_edge_inv/255.0;
  th_edge_inv.convertTo(th_edge_inv, CV_16U);

  int dilation_size = 1;
  cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1));
  cv::Mat element_dilate = getStructuringElement( cv::MORPH_ELLIPSE,
                       cv::Size( 2, 2 ),
                       cv::Point( dilation_size, dilation_size ) );
  cv::dilate( th_edge, th_edge, dilate_kernel );
  th_edge.convertTo(th_edge, CV_16U, 1 / 255.0); 

  curr_edge = interdephtImage_lin - curr_edge.mul((th_edge));


  //canny
  cv::Mat curr_edge = interdephtImage_lin.clone();
  cv::Mat detected_edges;
  int lowThreshold = 1;
  int ratio = 5;
  int kernel_size = 3;
  cv::blur( edge_frame, detected_edges, cv::Size(1,1) );
  cv::Canny( detected_edges, detected_edges, 1, 100 , kernel_size );
  th_edge =detected_edges;
  
  int dilation_size = 2;
  cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 5));
  cv::Mat element_dilate = getStructuringElement( cv::MORPH_ELLIPSE,
                       cv::Size( 2*dilation_size,  2*dilation_size ),
                       cv::Point( dilation_size, dilation_size ) );
  cv::dilate( th_edge, th_edge, dilate_kernel );

 // cv::morphologyEx(th_edge, th_edge, cv::MORPH_OPEN,  dilate_kernel,cv::Point(1,1),1);


  //th_edge.convertTo(th_edge, CV_16U, 1 / 255.0);  
  //cv::Mat th_edge_inv;
  //cv::bitwise_not(th_edge, th_edge_inv);
  //th_edge_inv = th_edge_inv/255.0;
  //th_edge_inv.convertTo(th_edge_inv, CV_16U);






  curr_edge = interdephtImage_lin - curr_edge.mul((th_edge));    

  
  

  //th_edge = th_edge.mul((detected_edges));
  //

 // curr_edge = interdephtImage - curr_edge;

  //Eigen::Matrix<float,Dynamic,Dynamic> edge_sz;
  //cv::cv2eigen(curr_edge,edge_sz);  



 // cv::Mat fft_filter = magI.mul(mask);

*/

  //===========================================fin filtrado por imagen=================================================
  /////////////////////////////

  // reconstruccion de imagen a nube 3D
  //============================================================================================================
  

  PointCloud::Ptr point_cloud (new PointCloud);
  PointCloud::Ptr cloud (new PointCloud);
  point_cloud->width = ZI.n_cols; 
  point_cloud->height = ZI.n_rows;
  point_cloud->is_dense = false;
  point_cloud->points.resize (point_cloud->width * point_cloud->height);

  arma::mat Zout = ZI;
  
  
  //////////////////filtrado de elementos interpolados con el fondo
  for (uint i=0; i< ZI.n_rows; i+=1)
   {       
      for (uint j=0; j<ZI.n_cols ; j+=1)
      {             
       if((ZI(i,j)== 0 ))
       {
        if(i+interpol_value<ZI.n_rows)
          for (int k=1; k<= interpol_value; k+=1) 
            Zout(i+k,j)=0;
        if(i>interpol_value)
          for (int k=1; k<= interpol_value; k+=1) 
            Zout(i-k,j)=0;
        }
      }      
    }

  //std::cout<<"Cols: "<<ZI.n_cols<<std::endl;
  //std::cout<<"Rows: "<<ZI.n_rows<<std::endl;

  ///////// imagen de rango a nube de puntos  
  int num_pc = 0; // numero de elementos en pointcloud
  for (uint i=0; i< ZI.n_rows; i+=1)
   {       
      for (uint j=0; j<ZI.n_cols ; j+=1)
      {

        float ang = M_PI-((2.0 * M_PI * j )/(ZI.n_cols));

        if (ang < min_FOV-M_PI/2.0|| ang > max_FOV - M_PI/2.0) 
          continue;

        // if(! (curr_edge.at<ushort>(i,j)== 0) ){
        ///////////////////////////// como imagen filtrada          

       /* float pc_modulo = (pow(2,16)- curr_edge.at<ushort>(i,j))*(max_depth)/pow(2,16);        
        float pc_x = sqrt(pow(pc_modulo,2)- pow(ZzI(i,j),2)) * cos(ang);
        float pc_y = sqrt(pow(pc_modulo,2)- pow(ZzI(i,j),2)) * sin(ang);        
        //edge_cloud->push_back(edge_x_data,edge_y_data,z_range(i,j);)
        point_cloud->points[num_pc].x = pc_x;
        point_cloud->points[num_pc].y = pc_y;
        point_cloud->points[num_pc].z = ZzI(i,j);*/
        ///////////////////////// con ZI        

        if(!(Zout(i,j)== 0 ))
        {  
          float pc_modulo = Zout(i,j);
          float pc_x = sqrt(pow(pc_modulo,2)- pow(ZzI(i,j),2)) * cos(ang);
          float pc_y = sqrt(pow(pc_modulo,2)- pow(ZzI(i,j),2)) * sin(ang);



          /// transformacion de resultado para correguir error de paralelismo con el suelo
          float ang_x_lidar = 0.6*M_PI/180.0;  

          Eigen::MatrixXf Lidar_matrix(3,3); //matrix de transforamcion para lidar a rango de imagen () se gira los angulos que tiene de error con respoecto al suelo
          Eigen::MatrixXf result(3,1);
          Lidar_matrix <<   cos(ang_x_lidar) ,0                ,sin(ang_x_lidar),
                            0                ,1                ,0,
                            -sin(ang_x_lidar),0                ,cos(ang_x_lidar) ;
    
               //   ,0   ,0               ,0                ,1;

          result << pc_x,
                    pc_y,
                    ZzI(i,j);
          
          result = Lidar_matrix*result;  // rotacion en eje X para correccion

          point_cloud->points[num_pc].x = result(0);
          point_cloud->points[num_pc].y = result(1);
          point_cloud->points[num_pc].z = result(2);

          /*
          point_cloud->points[num_pc].x = pc_x;
          point_cloud->points[num_pc].y = pc_y;
          point_cloud->points[num_pc].z = ZzI(i,j);
          */

          cloud->push_back(point_cloud->points[num_pc]); 

          num_pc++;
        }
      }
   }  

  //============================================================================================================

   PointCloud::Ptr P_out (new PointCloud);
 
   //filtrado de la nube
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50.0);
  sor.setStddevMulThresh (1.0);
  sor.filter (*P_out);*/



  // dowsmapling
  /*pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*P_out);*/



  P_out = cloud;


  Eigen::MatrixXf RTlc(4,4); // translation matrix lidar-camera
  RTlc<<   Rlc(0), Rlc(3) , Rlc(6) ,Tlc(0)
          ,Rlc(1), Rlc(4) , Rlc(7) ,Tlc(1)
          ,Rlc(2), Rlc(5) , Rlc(8) ,Tlc(2)
          ,0       , 0        , 0  , 1    ;

  //std::cout<<RTlc<<std::endl;

  int size_inter_Lidar = (int) P_out->points.size();

  Eigen::MatrixXf Lidar_camera(3,size_inter_Lidar);
  Eigen::MatrixXf Lidar_cam(3,1);
  Eigen::MatrixXf pc_matrix(4,1);
  Eigen::MatrixXf pointCloud_matrix(4,size_inter_Lidar);

  unsigned int cols = in_image->width;
  unsigned int rows = in_image->height;

  uint px_data = 0; uint py_data = 0;


  //point cloud con color 
  //PointCloud::Ptr pc_color (new PointCloud);
  pcl::PointXYZRGB point;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_color (new pcl::PointCloud<pcl::PointXYZRGB>);

 
  //pcl::PointXYZRGB pc_color;// (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZRGB>  pc_color;

   

  for (int i = 0; i < size_inter_Lidar; i++)
  {
      pc_matrix(0,0) = -P_out->points[i].y;
      pc_matrix(1,0) = -P_out->points[i].z;
      pc_matrix(2,0) =  P_out->points[i].x;
      pc_matrix(3,0) = 1.0;

      Lidar_cam = Mc * (RTlc * pc_matrix);

      px_data = (int)(Lidar_cam(0,0)/Lidar_cam(2,0));
      py_data = (int)(Lidar_cam(1,0)/Lidar_cam(2,0));
      
      if(px_data<0.0 || px_data>=cols || py_data<0.0 || py_data>=rows)
          continue;

      int color_dis_x = (int)(255*((P_out->points[i].x)/maxlen));
      int color_dis_z = (int)(255*((P_out->points[i].x)/10.0));
      if(color_dis_z>255)
          color_dis_z = 255;


      //point cloud con color
      cv::Vec3b & color = color_pcl->image.at<cv::Vec3b>(py_data,px_data);

     // std::cout<<" Y: "<<py_data;   std::cout<<" X: "<<px_data;
     // std::cout<<" Color: "<<color;

   

      point.x = P_out->points[i].x;
      point.y = P_out->points[i].y;
      point.z = P_out->points[i].z;
      

      point.r = (int)color[2]; 
      point.g = (int)color[1]; 
      point.b = (int)color[0];

      
      pc_color->points.push_back(point);   
      
      cv::circle(cv_ptr->image, cv::Point(px_data, py_data), 1, CV_RGB(255-color_dis_x,(int)(color_dis_z),color_dis_x),cv::FILLED);
      
    }
    pc_color->is_dense = true;
    pc_color->width = (int) pc_color->points.size();
    pc_color->height = 1;
    pc_color->header.frame_id = "velodyne";

  
 // sensor_msgs::ImagePtr image_msg;
 // image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", interdephtImage_lin).toImageMsg();
  
//  pcOnimg_pub.publish(image_msg);

  pcOnimg_pub.publish(cv_ptr->toImageMsg());

  
  //P_out->header.frame_id = "velodyne";
  pc_pub.publish (pc_color);
  auto t2= Clock::now();
 //std::cout<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()/1000000.0<<std::endl;
 // rate.sleep();
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "pontCloudOntImage");
  ros::NodeHandle nh;  
  

  /// Load Parameters

  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/max_ang_FOV", max_FOV);
  nh.getParam("/min_ang_FOV", min_FOV);
  nh.getParam("/pcTopic", pcTopic);
  nh.getParam("/imgTopic", imgTopic);

  XmlRpc::XmlRpcValue param;

  nh.getParam("/matrix_file/tlc", param);
  Tlc <<  (double)param[0]
         ,(double)param[1]
         ,(double)param[2];

  nh.getParam("/matrix_file/rlc", param);


  Rlc <<  (double)param[0] ,(double)param[1] ,(double)param[2]
         ,(double)param[3] ,(double)param[4] ,(double)param[5]
         ,(double)param[6] ,(double)param[7] ,(double)param[8];

  nh.getParam("/matrix_file/camera_matrix", param);

  Mc  <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3]
         ,(double)param[4] ,(double)param[5] ,(double)param[6] ,(double)param[7]
         ,(double)param[8] ,(double)param[9] ,(double)param[10],(double)param[11];

  message_filters::Subscriber<PointCloud2> pc_sub(nh, pcTopic , 1);
  message_filters::Subscriber<Image> img_sub(nh, imgTopic, 1);

  typedef sync_policies::ApproximateTime<PointCloud2, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, img_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  pcOnimg_pub = nh.advertise<sensor_msgs::Image>("/pcOnImage_image", 1);
  rangeImage = boost::shared_ptr<pcl::RangeImageSpherical>(new pcl::RangeImageSpherical);
  /*imgZ_pub = nh.advertise<sensor_msgs::Image>("/z_image", 10);*/

  pc_pub = nh.advertise<PointCloud> ("/points2", 1);  


  ros::spin();
  //return 0;
}

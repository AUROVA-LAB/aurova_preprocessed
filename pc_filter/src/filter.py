#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
import numpy as np
import fft
import time

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
          ]


pub_surf = rospy.Publisher("pc_surf", PointCloud2, queue_size=1)
pub_edge = rospy.Publisher("pc_edge", PointCloud2, queue_size=1)


header = Header()
header.frame_id = "/base_link"

z_data = None

def z_callback(img):
    z_data = (100.0/(2.0**16))*(img) - 50.0
    return z_data

def callback(data):

    start = time.time()

    max_depth = 100.0
    min_depth = 6
    br = CvBridge() 
    frame = br.imgmsg_to_cv2(data,"mono16")
    
    _, c = frame.shape
    current_frame = frame[:,0:int(c/2)]
    z_data = z_callback (frame[:,int(c/2):])

    frame_gray = (current_frame.copy()/2**8).astype(np.uint8)   

    frame_gray_eq = cv2.equalizeHist(frame_gray)
    canny_prevoo = cv2.Sobel(frame_gray_eq,cv2.CV_64F,0,1,ksize=1)
    canny_prevoo = np.absolute(canny_prevoo)    
    canny_prevoosobel = np.uint8(canny_prevoo)

    _, canny0 = cv2.threshold((canny_prevoosobel.astype(np.uint8)),0,255,cv2.THRESH_BINARY_INV)

   # kernel1 = np.array(([[2, 1, 2], [1, 2, 1], [2, 1, 2]]),np.uint8)
    kernel = np.ones((3,3),np.uint8)
    canny0 = cv2.morphologyEx(canny0, cv2.MORPH_CLOSE, kernel,iterations = 2)    

    #frame_gray_cu = frame_gray*(canny0/255) 
    #curr_fft = current_frame*(canny0/255) 
  

    #canny_prev_1 = cv2.Canny(frame_gray,10,40,True)
    #_, canny = cv2.threshold((canny_prev_1.astype(np.uint8)),0,255,cv2.THRESH_BINARY)


    canny_prevoo = cv2.Sobel(frame_gray*(canny0/255),cv2.CV_64F,1,0,ksize=1)
    canny_prevoo = np.absolute(canny_prevoo)    
    canny_prevoo = np.uint8(canny_prevoo)
    _, canny = cv2.threshold((canny_prevoo.astype(np.uint8)),5,255,cv2.THRESH_BINARY)

    #zimg =cv2.equalizeHist(z_data.astype(np.uint8))
    #_,zimg = cv2.threshold(255- (z_data.astype(np.uint8)),200,255,cv2.THRESH_BINARY)

    #conflict_points =  (conflict_points - canny)

    #frame_gray_cu = frame_gray_cu * (conflict_points/255)   # con esto uso lo de los puntos conflictivos

    ##conflict_points = cv2.Canny(frame_gray_eq, 1,2,True)
    #_, conflict_points = cv2.threshold((canny_prev.astype(np.uint8)),0,255,cv2.THRESH_BINARY_INV)
   # kernel = np.array(([[0, 1, 0], [0, 1, 0], [0, 1, 0]]),np.uint8)
    #canny = cv2.dilate(canny,kernel,iterations = 1)

   # #canny = (canny_prevoo/255) * canny_ant
    #canny = canny_prevoo


    #kernel = np.ones((1,3),np.uint8)
    #canny = cv2.dilate(canny,kernel,iterations = 1)  

    curr_edge = current_frame * (canny/255)  
    #curr_edge = current_frame *(canny0/255)  
    
    curr_surf = (current_frame - curr_edge) *((255-canny0)/255)   ## con suelo
       
    #curr_surf = (current_frame - curr_edge)*(canny0/255) ## sin suelo

    #curr_edge = current_frame -curr_surf
    
    #curr_surf = curr_surf+curr_edge

    ang_mat = np.tile(np.arange(180,-180,-360.0/float(current_frame.shape[1])),(current_frame.shape[0],1))
    
    ## eliminacion de 0
    edge_sz = curr_edge.astype(np.float)
    ##parte para eliminar los punots cercanos al lidar
    edge_sz [(edge_sz > 2**16 - (min_depth/max_depth) * 2**16)] = 2**16
    edge_sz[edge_sz == 0] = 2**16

    surf_sz = curr_surf.astype(np.float)
    #surf_sz [(surf_sz > 2**16 - (min_depth/max_depth) * 2**16)] = 2**16
    surf_sz[surf_sz == 0] = 2**16

    edge_sz = (2**16-edge_sz)*(max_depth)/2**16 
    surf_sz = (2**16-surf_sz)*(max_depth)/2**16 

    x_edge = (( np.sqrt((edge_sz)**2 - (z_data)**2)* np.cos(ang_mat*np.pi/180.0)))
    y_edge = (( np.sqrt((edge_sz)**2 - (z_data)**2)* np.sin(ang_mat*np.pi/180.0)))

    x_surf =  np.sqrt((surf_sz)**2 - (z_data)**2)* np.cos(ang_mat*np.pi/180.0)
    y_surf =  np.sqrt((surf_sz)**2 - (z_data)**2)* np.sin(ang_mat*np.pi/180.0)


    z_surf = z_data [0:len(z_data):2]
    x_surf = x_surf [0:len(x_surf):2]
    y_surf = y_surf [0:len(y_surf):2]

   # z_surf = z_data [0:len(z_data):2]  # para VLP16
   # x_surf = x_surf [0:len(x_surf):2] # para VLP16
   # y_surf = y_surf [0:len(y_surf):2] # para VLP16

    z_edge = z_data[0:len(z_data):1]
    x_edge = x_edge [0:len(x_edge):1]
    y_edge = y_edge [0:len(y_edge):1]

    z_surf = z_surf.flatten()
    z_edge = z_edge.flatten()
    #z_edge = z_data.flatten()
    x_surf = x_surf.flatten()
    y_surf = y_surf.flatten()
    x_edge = x_edge.flatten()
    y_edge = y_edge.flatten()



   # indices = np.argsort(a)

    bool_surf = np.logical_or(np.absolute(y_surf) >0, np.absolute(x_surf) >0).astype(int)
    bool_edge = np.logical_or(np.absolute(y_edge) >0, np.absolute(x_edge) >0).astype(int)

    z_surf = z_surf * bool_surf
    z_edge = z_edge * bool_edge


    

    #bool_delsurf = np.logical_and(np.absolute(y_surf) >0, np.absolute(x_surf) >0 ,np.absolute(z_surf)>0).astype(int)
    #bool_deledge = np.logical_and(np.absolute(y_edge) >0, np.absolute(x_edge) >0 ,np.absolute(z_edge)>0).astype(int)

    
   
    pt_surf  = np.array([x_surf,y_surf,z_surf ,np.zeros(z_surf.shape)])
    pt_edge  = np.array([x_edge,y_edge,z_edge ,np.zeros(z_edge.shape)])

    
 
    points_surf = np.transpose(pt_surf).tolist()
    points_edge = np.transpose(pt_edge).tolist()   

    pcl_surf = point_cloud2.create_cloud(header, fields, points_surf)
    pcl_edge = point_cloud2.create_cloud(header, fields, points_edge)

    pub1 = rospy.Publisher('/depthimg', Image, queue_size=1)
    #pub2.publish(br.cv2_to_imgmsg(((curr_fft/2**8).astype(np.uint8))))
    #pub.publish(br.cv2_to_imgmsg(img_gray.astype(np.uint8)))

    #pc_curr = point_cloud2.create_cloud(header, fields, points_curr)
    #pc_edge = point_cloud2.create_cloud(header, fields, points_edge)
    #pc_surf = point_cloud2.create_cloud(header, fields, points_surf)


    pub1.publish(br.cv2_to_imgmsg(curr_edge))
    #pub2.publish(br.cv2_to_imgmsg(conflict_points))
  
   # pc_curr.header.stamp = rospy.Time.now()
  #  pc_edge.header.stamp = rospy.Time.now()
   # pc_surf.header.stamp = rospy.Time.now()
    #pub_filt.publish(pc_curr)

    pub_edge.publish(pcl_edge)
    pub_surf.publish(pcl_surf)

    print((time.time()-start)*1000.0)
      
def receive_message():    
 
    rospy.init_node('img_process_pub', anonymous=True)
    rospy.Subscriber('/depth_image', Image, callback)

    rospy.spin()

if __name__ == '__main__':
    receive_message()
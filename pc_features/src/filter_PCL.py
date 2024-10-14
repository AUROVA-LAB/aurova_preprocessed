#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
import numpy as np
import fft

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
          ]


pub_surf = rospy.Publisher("pcl_surf", PointCloud2, queue_size=1)
pub_edge = rospy.Publisher("pcl_edge", PointCloud2, queue_size=1)
pub_suelo = rospy.Publisher("pcl_ground", PointCloud2, queue_size=1)

header = Header()
header.frame_id = "/velodyne"

z_data = None

def z_callback(img):
    z_data = (100.0/(2.0**16))*(img) - 50.0
    return z_data

def callback(data):
    max_depth = 100.0
    min_depth = 0.0
 
    br = CvBridge() 
    frame = br.imgmsg_to_cv2(data,"mono16")
    
    _, c = frame.shape
    current_frame = frame[:,0:int(c/2)]
    z_data = z_callback (frame[:,int(c/2):])
    frame_gray = (current_frame.copy()/2**8).astype(np.uint8)

    ##### MASK
    rows, cols = frame_gray.shape

    ## ground mask      
    limits_cols = int(round(cols/2))
    ground_mask = np.ones((rows, cols, 2), np.uint8)
    ground_mask[:,limits_cols-1:limits_cols+2] = 0

    # ground points extraction with FFT
    mask,_,curr_fft = fft.fourier_filtter(current_frame, ground_mask,1)          
    frame_gray_cu = (curr_fft.copy()/2**8).astype(np.uint8) 
    canny_prev = cv2.Canny(frame_gray_cu, 100,255)
    _, canny = cv2.threshold((canny_prev.astype(np.uint8)),0,255,cv2.THRESH_BINARY)   

    ## features filtered
    curr_edge = curr_fft * (canny/255)
    curr_surf = curr_fft - curr_edge 
    curr_suelo = current_frame - curr_surf - curr_edge

    ang_mat = np.tile(np.arange(180,-180,-360.0/float(curr_fft.shape[1])),(curr_fft.shape[0],1))
    
    ## deleted 0 values in pixels 
    edge_sz = curr_edge.astype(np.float)
    edge_sz[edge_sz == 0] = 2**16
    surf_sz = curr_surf.astype(np.float)
    surf_sz[surf_sz == 0] = 2**16
    ground_sz = curr_suelo.astype(np.float)
    ground_sz[ground_sz == 0] = 2**16

    edge_sz   = (2**16-edge_sz)
    surf_sz   = (2**16-surf_sz)
    ground_sz = (2**16-ground_sz)


    x_edge = (( np.sqrt((edge_sz*(max_depth)/2**16)**2 - (z_data)**2)* np.cos(ang_mat*np.pi/180.0)))
    y_edge = (( np.sqrt((edge_sz*(max_depth)/2**16)**2 - (z_data)**2)* np.sin(ang_mat*np.pi/180.0)))

    x_surf = (( np.sqrt((surf_sz*(max_depth)/2**16)**2 - (z_data)**2)* np.cos(ang_mat*np.pi/180.0)))
    y_surf = (( np.sqrt((surf_sz*(max_depth)/2**16)**2 - (z_data)**2)* np.sin(ang_mat*np.pi/180.0)))

    x_ground = (( np.sqrt((ground_sz*(max_depth)/2**16)**2 - (z_data)**2)* np.cos(ang_mat*np.pi/180.0)))
    y_ground = (( np.sqrt((ground_sz*(max_depth)/2**16)**2 - (z_data)**2)* np.sin(ang_mat*np.pi/180.0)))

    z_surf = z_data[0:len(z_data):2]
    x_surf = x_surf [0:len(x_surf):2]
    y_surf = y_surf [0:len(y_surf):2]

    z_suelo = z_data[0:len(z_data):2]
    x_ground = x_ground [0:len(x_ground):2]
    y_ground = y_ground [0:len(y_ground):2]

    z_surf = z_surf.flatten()
    z_edge = z_data.flatten()
    x_surf = x_surf.flatten()
    y_surf = y_surf.flatten()
    x_edge = x_edge.flatten()
    y_edge = y_edge.flatten()

    z_suelo = z_suelo.flatten()
    x_ground = x_ground.flatten()
    y_ground = y_ground.flatten()
   
    bool_surf = np.logical_or(np.absolute(y_surf) > min_depth, np.absolute(x_surf) > min_depth).astype(int)
    bool_edge = np.logical_or(np.absolute(y_edge) > min_depth, np.absolute(x_edge) > min_depth).astype(int)
    bool_ground = np.logical_or(np.absolute(y_ground) > min_depth, np.absolute(x_ground) > min_depth).astype(int)


    pt_surf  = np.array([x_surf,y_surf,z_surf * bool_surf,np.zeros(z_surf.shape)])
    pt_edge  = np.array([x_edge,y_edge,z_edge * bool_edge,np.zeros(z_edge.shape)])
    pt_ground = np.array([x_ground,y_ground,z_suelo * bool_ground,np.zeros(z_suelo.shape)])
 
    points_surf = np.transpose(pt_surf).tolist()
    points_edge = np.transpose(pt_edge).tolist()
    points_ground = np.transpose(pt_ground).tolist() 

    pub2 = rospy.Publisher('original_image', Image, queue_size=1)
    pub1 = rospy.Publisher('mask', Image, queue_size=1)
    pub = rospy.Publisher('filter', Image, queue_size=1)

    pc_ground= point_cloud2.create_cloud(header, fields, points_ground)
    pc_edge = point_cloud2.create_cloud(header, fields, points_edge)
    pc_surf = point_cloud2.create_cloud(header, fields, points_surf)

    pub2.publish(br.cv2_to_imgmsg(current_frame))
    pub1.publish(br.cv2_to_imgmsg(mask))
    pub.publish(br.cv2_to_imgmsg(curr_fft))

    pc_ground.header.stamp = rospy.Time.now()
    pc_edge.header.stamp = rospy.Time.now()
    pc_surf.header.stamp = rospy.Time.now()
    pub_suelo.publish(pc_ground)
    pub_edge.publish(pc_edge)
    pub_surf.publish(pc_surf)


      
def receive_message():    
 
    rospy.init_node('img_process_pub', anonymous=True)
    rospy.Subscriber('/depth_z_image', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    receive_message()

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
import numpy as np
import time
import math
import icp
import fft
import matplotlib.pyplot as plt
import numpy.ma as ma


import struct
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
          ]


pub_filt = rospy.Publisher("pc_filt", PointCloud2, queue_size=2)
pub_surf = rospy.Publisher("pc_surf", PointCloud2, queue_size=2)
pub_edge = rospy.Publisher("pc_edge", PointCloud2, queue_size=2)


header = Header()
header.frame_id = "/base_link"
rgb = struct.unpack('I', struct.pack('BBBB', 128, 128, 128, 255))[0]



lk_params = dict(winSize  = (50, 50),
                maxLevel = 255,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.003))

feature_params = dict(maxCorners = 20,
                    qualityLevel = 0.01,
                    minDistance = 0,
                    blockSize = 1)


trajectory_len = 5
detect_interval = 1
trajectories = []
frame_idx = 0
prev_gray = 0
#x_prev = 0
#y_prev = 0
Rsal = 0
tsal = 0
H_s = np.eye(3)
distancia = 50.0

flag = False
previous_points =0

def callback(data):
    start = time.time()
    global trajectories, prev_gray, frame_idx, Rsal, tsal , H_s, flag, previous_points#,x_prev, y_prev,
 
    br = CvBridge() 
    current_frame = br.imgmsg_to_cv2(data,"mono16")
   
    #frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    frame_gray = (current_frame.copy()/2**8).astype(np.uint8)

    #frame_x = cv2.flip(current_frame, 1)
    #frame_y =  cv2.flip(current_frame, 1)



    img = cv2.cvtColor(current_frame/2**8,cv2.COLOR_GRAY2RGB)
    img = img.astype(np.uint8)
    
    #img_gray = cv2.cvtColor(img/2**8,cv2.COLOR_RGB2GRAY)
    #img_gray = (img).astype(np.uint8)



    ##### Mascaras
    rows, cols = frame_gray.shape

    ## Mascara para quitar el suelo        
    limits_cols = int(round(cols/2))
    limits_rows = int(round(rows/2))
    mask_suelo = np.ones((rows, cols, 2), np.uint8)
    mask_suelo[:,limits_cols:limits_cols+1] = 0

    ## Mascara para identificar objetos        
    limits_cols = int(round(cols/2))
    limits_rows = int(round(rows/2))
    mask_obj = np.zeros((rows, cols, 2), np.uint8)
    mask_obj[limits_rows:limits_rows+1,:] = 1

    # ground points extraction with FFT
    magnitude_spectrum,mask,curr_fft = fft.fourier_filtter(current_frame, mask_suelo,2)    
    magnitude_spectrum,mask_2,_ = fft.fourier_filtter(curr_fft, mask_obj,50)    
    #curr_fft = curr_fft * mask_2
    
    frame_gray_cu = (curr_fft.copy()/2**8).astype(np.uint8) 
    #frame_gray_cu = mask_2 
    #frame_gray_cu = cv2.GaussianBlur(frame_gray_cu, (5,5), 0)

    #kernel = np.ones((3,3),np.uint8)
    #frame_gray_cu = cv2.morphologyEx(frame_gray_cu, cv2.MORPH_OPEN, kernel)

    canny = cv2.Canny(frame_gray_cu, 5,5)
    ret, canny = cv2.threshold((canny.astype(np.uint8)),128,255,cv2.THRESH_BINARY)
    kernel = np.ones((3,3),np.uint8)
    dilation = cv2.dilate(canny,kernel,iterations = 1)
    conflict_points = cv2.Sobel(curr_fft,cv2.CV_16U,1,0,ksize=3)
    conflict_points = np.absolute(conflict_points)

   
    ret, conflict_points = cv2.threshold((conflict_points.astype(np.uint8)),128,255,cv2.THRESH_BINARY_INV)

    #conflict_points = 255

  

    curr_fft = curr_fft * conflict_points/255   
    curr_edge = curr_fft * (canny/255)
    curr_surf = curr_fft - curr_edge

   
   # img_mask =(curr_fft/2**8).astype(np.uint8)

   # while(max_actual * fact >= 0):   

        #maximos = np.append(maximos,max_actual)
  
       # hist = np.delete(hist, max_actual)
       # max_actual = np.argmax(hist)
        

        #print("Actual:",max_actual)
        #print("Anterior:", max_anter)

    #    ret, thresh = cv2.threshold(img_mask,max_actual,255,cv2.THRESH_BINARY_INV)
    #    img_mask = img_mask*(thresh/255)
    #    max_anter = max_actual
    #    max_actual = max_actual -50
        
    #    plt.plot(hist)        
       # pub = rospy.Publisher('video_frames', Image, queue_size=10)
       # pub.publish(br.cv2_to_imgmsg(thresh))
       # plt.show()
        
    
   


      
    #fast = cv2.FastFeatureDetector_create()
    # find and draw the keypoints
    #kp = fast.detect(img,None)
    #img = cv2.drawKeypoints(img, kp, None, color=(255,0,0))

    #fast.setNonmaxSuppression(0)
    #kp = fast.detect(img, None)
    #print( "Total Keypoints without nonmaxSuppression: {}".format(len(kp)) )
    #img = cv2.drawKeypoints(img, kp, None, color=(0,0,255))


    
    if(0):
        ## filtrado de objetos
        img_u8 = (curr_fft/2**8).astype(np.uint8)   
        sobelx = cv2.Sobel(img_u8,cv2.CV_64F,1,0,ksize=5)
        sobely = cv2.Sobel(img_u8,cv2.CV_64F,0,1,ksize=5)
        bmsk = 255*(np.sqrt(sobelx**2+sobely**2) > 5) 
        bmsk = bmsk.astype(np.uint8)
        kernel = np.ones((3,3),np.uint8)
        dilation = cv2.dilate(bmsk,kernel,iterations = 1)
        contours, hierarchy = cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        dilation = cv2.cvtColor(dilation,cv2.COLOR_GRAY2RGB)

        contours_poly = [None]*len(contours)
        boundRect = [None]*len(contours)
        centers = [None]*len(contours)
        radius = [None]*len(contours)

        for i, c in enumerate(contours):
            contours_poly[i] = cv2.approxPolyDP(c, 3, True)
            boundRect[i] = cv2.boundingRect(contours_poly[i])
            centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
        
        print("Cantidad:",len(boundRect))
        for i in range(len(contours)):
            print("Rectangulos:",boundRect[i])
            
        
            #cv2.drawContours(img, contours_poly, i, (0,255,0))
        #  if(boundRect[i][2]>10 or boundRect[i][3]>10):
            cv2.rectangle(img, (int(boundRect[i][0]), int(boundRect[i][1])),
            (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])),  (0,255,0), 2)
            #   cv2.putText(img, '%d' % i, (int(boundRect[i][0]), int(boundRect[i][1])), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,255), 1)
        # if i>1:
            #    break

    


       
    try:
        _,_,prev_fft = fft.fourier_filtter(prev_gray, mask_suelo,5)
        #prev_fft = 2**16 - frame_gray
    except:
       prev_fft = curr_fft



    ang_mat = np.tile(np.arange(180,-180,-360.0/float(curr_fft.shape[1])),(curr_fft.shape[0],1))
    
    ## eliminacion de 0
    curr_sz = curr_fft.astype(np.float)
    curr_sz[curr_sz == 0] = 2**16

    edge_sz = curr_edge.astype(np.float)
    edge_sz[edge_sz == 0] = 2**16

    surf_sz = curr_surf.astype(np.float)
    surf_sz[surf_sz == 0] = 2**16
    #prev_sz = prev_mean.copy()
    #prev_sz [prev_sz <= 2**16*(10/255.0)] = 2**16
       
    #x_curr = np.zeros((curr_fft.shape[0],curr_fft.shape[1]),dtype=np.uint16)
    x_curr = ((2**16-curr_sz) * np.cos(ang_mat*np.pi/180.0))*distancia/2**16

    #y_curr = np.zeros((curr_fft.shape[0],curr_fft.shape[1]),dtype=np.uint16)
    y_curr = ((2**16-curr_sz) * np.sin(ang_mat*np.pi/180.0))*distancia/2**16

    #x_edge = np.zeros((curr_edge.shape[0],curr_edge.shape[1]),dtype=np.uint16)
    x_edge = ((2**16-edge_sz) * np.cos(ang_mat*np.pi/180.0))*distancia/2**16

    #y_edge = np.zeros((curr_edge.shape[0],curr_edge.shape[1]),dtype=np.uint16)
    y_edge = ((2**16-edge_sz) * np.sin(ang_mat*np.pi/180.0))*distancia/2**16

    #x_surf = np.zeros((curr_surf.shape[0],curr_surf.shape[1]),dtype=np.uint16)
    x_surf = ((2**16-surf_sz) * np.cos(ang_mat*np.pi/180.0))*distancia/2**16

    #y_surf = np.zeros((curr_surf.shape[0],curr_surf.shape[1]),dtype=np.uint16)
    y_surf = ((2**16-surf_sz) * np.sin(ang_mat*np.pi/180.0))*distancia/2**16



    #imgY_prev = y_curr.astype(np.uint8)

    #x_prev = np.zeros((prev_fft.shape[0],curr_fft.shape[1]),dtype=np.uint16)
    #x_prev = ((2**16-prev_sz) * np.cos(ang_mat*np.pi/180.0))
    #imgX_prev = x_prev.astype(np.uint8)

    #y_prev = np.zeros((prev_fft.shape[0],curr_fft.shape[1]),dtype=np.uint16)
    #y_prev = ((2**16-prev_sz) * np.sin(ang_mat*np.pi/180.0))




    points_curr = []
    points_surf = []
    points_edge = []

    x_points_curr = []
    y_points_curr = []
    z_points_curr = []
    
    lim_y = x_edge.shape[0]

    lim = 8
    for z in range(lim_y):      
        for xx in range(x_edge.shape[1]):
            y_c = y_curr[lim_y-1-z,xx]
            x_c = x_curr[lim_y-1-z,xx]    

            y_e = y_edge[lim_y-1-z,xx]
            x_e = x_edge[lim_y-1-z,xx]

            y_s = y_surf[lim_y-1-z,xx]
            x_s = x_surf[lim_y-1-z,xx]                       
                
            

            z_c = math.sqrt(x_c**2+y_c**2) * math.tan((z-lim_y/2)*(0.0349066))
            z_s = math.sqrt(x_s**2+y_s**2) * math.tan((z-lim_y/2)*(0.0349066))
            z_e = math.sqrt(x_e**2+y_e**2) * math.tan((z-lim_y/2)*(0.0349066))

            r = int(abs(x_e/distancia)*255.0)
            g = int(abs(y_e/distancia)* 255.0)
            b = int(z_e* 255.0)
            a = 255

            #print (r, g, b, a)
            rgb = struct.unpack('I', struct.pack('BBBB', 128, 128, 128, a))[0]
            # print (hex(rgb))
            pt_curr = [x_c, y_c, z_c, rgb]
            pt_surf = [x_s, y_s, z_s, rgb]
            pt_edge = [x_e, y_e, z_e, rgb]

            points_curr.append(pt_curr)
            points_surf.append(pt_surf)
            points_edge.append(pt_edge)

          #  x_points_curr.append(x) 
          #  y_points_curr.append(y) 
          #  z_points_curr.append(0.5*z) 


   
    frame_idx += 1
    prev_gray = frame_gray_cu

    # End time
    end = time.time()
    # calculate the FPS for current frame detection
    fps = 1 / (end-start)

    

    #curr_mean = curr_mean.astype(np.uint8)

    pub2 = rospy.Publisher('video_frames', Image, queue_size=1)
    #pub.publish(br.cv2_to_imgmsg(((curr_fft/2**8).astype(np.uint8))))
    #pub.publish(br.cv2_to_imgmsg(img_gray.astype(np.uint8)))

   


    pc_curr = point_cloud2.create_cloud(header, fields, points_curr)
    pc_edge = point_cloud2.create_cloud(header, fields, points_edge)
    pc_surf = point_cloud2.create_cloud(header, fields, points_surf)


    pub2.publish(br.cv2_to_imgmsg(curr_fft))
    #pub2.publish(br.cv2_to_imgmsg(new_image_x))
    #frame_gray_cu
    #print(np.max(img_gray))

  

    pc_curr.header.stamp = rospy.Time.now()
    pc_edge.header.stamp = rospy.Time.now()
    pc_surf.header.stamp = rospy.Time.now()
    pub_filt.publish(pc_curr)
    pub_edge.publish(pc_edge)
    pub_surf.publish(pc_surf)
    

      
def receive_message():
 
    rospy.init_node('img_process_pub', anonymous=True)   
    rospy.Subscriber('/depth_image', Image, callback)
    rospy.spin()
    
    cv2.destroyAllWindows()
  
if __name__ == '__main__':
    receive_message()





 

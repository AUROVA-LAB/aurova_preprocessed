#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
import numpy as np
import fft
import matplotlib.pyplot as plt
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


pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

header = Header()
header.frame_id = "/velodyne"
rgb = struct.unpack('I', struct.pack('BBBB', 128, 128, 128, 255))[0]



Rsal = 0
tsal = 0
H_s = np.eye(3)
fact = 0.9
max_anter = 255

prev_fft = np.zeros((21,720))

def callback(data):

    global trajectories, prev_gray, frame_idx, Rsal, tsal , H_s,max_anter , prev_fft
 
    br = CvBridge() 
    current_frame = br.imgmsg_to_cv2(data,"mono16")
    frame_gray = (current_frame.copy()/2**8).astype(np.uint8)

    #imagen en RGB para plot
    img = cv2.cvtColor(current_frame/2**8,cv2.COLOR_GRAY2RGB)
    img = img.astype(np.uint8)
    
    ##### Mascaras
    rows, cols = frame_gray.shape

    #############################################Filtrado con fourier#########################################
    ## Mascara para quitar el suelo        
    limits_cols = int(round(cols/2))
    limits_rows = int(round(rows/2))
    mask_suelo = np.ones((rows, cols, 2), np.uint8)
    mask_suelo[:,limits_cols-2:limits_cols+3] = 0

    ## Mascara para identificar objetos        
    limits_cols = int(round(cols/2))
    limits_rows = int(round(rows/2))
    mask_obj = np.zeros((rows, cols, 2), np.uint8)
    mask_obj[limits_rows:limits_rows+1,:] = 1

    # ground points extraction with FFT
    _,_,curr_fft = fft.fourier_filtter(current_frame, mask_suelo,5)    
    _,mask2,_ = fft.fourier_filtter(curr_fft, mask_obj,20)    
    #curr_fft = curr_fft * mask_2
    frame_gray_cu = (curr_fft.copy()/2**8).astype(np.uint8) 

    hist = cv2.calcHist([frame_gray_cu], [0], None, [256], [0, 256])


    ##########################################################################################################
    


    ######################################promedios############################################
    curr_nan = curr_fft.astype(np.float)
    curr_nan[curr_nan == 0] = np.nan
    curr_mean = np.nanmean(curr_nan,0)   
    curr_mean[(np.isnan(curr_mean))] = 0
    
    prev_nan = prev_fft.astype(np.float)
    prev_nan[prev_nan == 0] = np.nan
    prev_mean = np.nanmean(prev_nan,0)
    prev_mean[(np.isnan(prev_mean))] = 0

    ########################################################################################


    ang_mat = np.tile(np.arange(270,-90,-360.0/float(curr_fft.shape[1])),(curr_fft.shape[0],1))



   ## eliminacion de 0
    curr_sz = curr_fft.astype(np.float)
    curr_sz[curr_sz == 0] = 2**16

    #prev_sz = prev_mean.copy()
    #curr_sz [curr_sz <= 2**16*(10/255.0)] = 2**16
    #prev_sz [prev_sz <= 2**16*(10/255.0)] = 2**16
       
    x_curr = np.zeros((curr_fft.shape[0],curr_fft.shape[1]),dtype=np.uint16)
    x_curr = ((2**16-curr_sz) * np.cos(ang_mat*np.pi/180.0))

    y_curr = np.zeros((curr_fft.shape[0],curr_fft.shape[1]),dtype=np.uint16)
    y_curr = ((2**16-curr_sz) * np.sin(ang_mat*np.pi/180.0))

    #imgY_prev = y_curr.astype(np.uint8)

   # x_prev = np.zeros((prev_fft.shape[0],curr_fft.shape[1]),dtype=np.uint16)
   # x_prev = ((2**16-prev_sz) * np.cos(ang_mat*np.pi/180.0))
    #imgX_prev = x_prev.astype(np.uint8)

    #y_prev = np.zeros((prev_fft.shape[0],curr_fft.shape[1]),dtype=np.uint16)
    #y_prev = ((2**16-prev_sz) * np.sin(ang_mat*np.pi/180.0))


    ################# Matrices aplanadas
    '''
    x_data = np.arange(0,curr_mean.shape[0],1) ## axis X for all matrix
    y_x_curr = np.reshape(x_curr,(1,x_curr.shape[0]*x_curr.shape[1]))
    y_y_curr = np.reshape(y_curr,(1,y_curr.shape[0]*y_curr.shape[1]))

    y_x_prev = np.reshape(x_prev,(1,x_prev.shape[0]*x_prev.shape[1]))
    y_y_prev = np.reshape(y_prev,(1,y_prev.shape[0]*y_prev.shape[1]))

    ran_dt = 5
    y_x_curr = y_x_curr[0][::ran_dt]
    y_y_curr = y_y_curr[0][::ran_dt]
    y_x_prev = y_x_prev[0][::ran_dt]
    y_y_prev = y_y_prev[0][::ran_dt]
    current_points  = np.vstack((x_data, curr_mean))
    previous_points = np.vstack((x_data, prev_mean))
    '''
    
   
    if (1):
        plt.plot(x_curr[::], y_curr[::], 'b.')
       # plt.plot(x_data[::], prev_mean[::], 'r,')        
        #plt.plot(x_prev, y_prev, 'r.')
        plt.show()


    prev_fft = curr_fft
    

  

    curr_mean = curr_mean.astype(np.uint8)

    pub = rospy.Publisher('video_frames', Image, queue_size=10)
    pub.publish(br.cv2_to_imgmsg(((curr_fft/2**8).astype(np.uint8))))
    #pub.publish(br.cv2_to_imgmsg(img_gray.astype(np.uint8)))
    #pub.publish(br.cv2_to_imgmsg(img))
    #print(np.max(img_gray))

       
def receive_message():
 
    rospy.init_node('img_process_pub', anonymous=True)   
    rospy.Subscriber('/depth_image', Image, callback)
    rospy.spin()
    
    cv2.destroyAllWindows()
  
if __name__ == '__main__':
    receive_message()





 

#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from matplotlib import pyplot as plt
 
def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data,"mono16")
  f = np.fft.fft2(current_frame)
  fshift = np.fft.fftshift(f)
  magnitude_spectrum = 20*np.log(np.abs(fshift))
  #current_frame = cv2.circle(current_frame, (1,1), 10, (255,0,255), 5)
  #magnitude_spectrum
  #np.fft.ifft2(a)

  #
  plt.subplot(121),plt.imshow(current_frame, cmap = 'gray')
  plt.title('Input Image'), plt.xticks([]), plt.yticks([])
  plt.subplot(122),plt.imshow(magnitude_spectrum, cmap = 'gray')
  plt.title('Magnitude Spectrum'), plt.xticks([]), plt.yticks([])
  plt.show()
  
  pub = rospy.Publisher('video_frames', Image, queue_size=10)
  pub.publish(br.cv2_to_imgmsg(np.abs(fshift))) 

  print("DATOS:")
  print(magnitude_spectrum)
  # Display image
 

  #cv2.imshow("camera", current_frame)
  #cv2.imshow("fourier", np.abs(fshift),cmap = 'gray')
   
  #cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('img_process_pub', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('/depth_image', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()

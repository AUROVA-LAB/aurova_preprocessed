#!/usr/bin/env python3

import message_filters
import rospy
import sys
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(range_in, signal_in, reflec_in, nearir_in):
    global flag, bridge, merge_img_pub, cont, i,path,j
    if flag:
        flag=False
        range = bridge.imgmsg_to_cv2(range_in)
        range = (range/256).astype('uint8')
        signal = bridge.imgmsg_to_cv2(signal_in)
        signal =(signal/256).astype('uint8')
        reflec = bridge.imgmsg_to_cv2(reflec_in)
        reflec =(reflec/256).astype('uint8')
        nearir = bridge.imgmsg_to_cv2(nearir_in)
        nearir =(nearir/256).astype('uint8')

        merged_4ch=cv2.merge([signal,nearir,reflec,range])
        merged_3ch=cv2.merge([signal,nearir,reflec])

        image_message = bridge.cv2_to_imgmsg(merged_4ch, "bgra8")  
        image_message.header.stamp = range_in.header.stamp
        image_message.header.frame_id = "ouster_sensor"
        merge_img_pub.publish(image_message)
        
        if (cont%10==0):            
            if cv2.imwrite(path+"/4_channels/merged_"+str(i)+".png", merged_4ch):
                print("imagen merged_"+str(i)+".png 4 channels Guardada")
                i = i+1
            else:
                print("[ERROR] Imagen 4 channles no guardada. verificar el path: "+path)
            if cv2.imwrite(path+"/3_channels/merged_"+str(j)+".png", merged_3ch):
                print("imagen merged_"+str(j)+".png 3 channels Guardada")
                j = j+1
            else:
                print("[ERROR] Imagen 3 channles no guardada. verificar el path: "+path)

            cont = 0
        cont= cont+1


if __name__ == '__main__':
    global flag, bridge, merge_img_pub, cont, i,path,j
    path = "../../../../labrobotica/dataset/merged/"
    cont = 0
    i=0
    j=0
    try:
        os.mkdir(path+"4_channels")
        os.mkdir(path+"3_channels")
    except:
        print("[ERROR] El path "+path+"no existe, o las carpetas 4 y 3 channels ya han sido creadas anteriormente")

    rospy.init_node("sync_images")
    flag=False
    bridge = CvBridge()
    range_sub = message_filters.Subscriber('/ouster/range_image', Image)
    signal_sub = message_filters.Subscriber('/ouster/signal_image', Image)
    reflec_sub = message_filters.Subscriber('/ouster/reflec_image', Image)
    nearir_sub = message_filters.Subscriber('/ouster/nearir_image', Image)
    merge_img_pub = rospy.Publisher('/ouster_merged', Image, queue_size=10)

    ts = message_filters.TimeSynchronizer([range_sub, signal_sub, reflec_sub, nearir_sub], 10)
    ts.registerCallback(callback)

    r = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
        flag=True
        r.sleep()

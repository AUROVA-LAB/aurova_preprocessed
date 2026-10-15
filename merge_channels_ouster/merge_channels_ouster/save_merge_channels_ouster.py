#!/usr/bin/env python3

import message_filters
import rclpy
import sys
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(range_in, signal_in, reflec_in, nearir_in):
    global bridge, merge_img_pub, cont, i, path, j, offset, freq

    depth = bridge.imgmsg_to_cv2(range_in)
    depth = (depth/256).astype('uint8')
    signal = bridge.imgmsg_to_cv2(signal_in)
    signal =(signal/256).astype('uint8')
    reflec = bridge.imgmsg_to_cv2(reflec_in)
    reflec =(reflec/256).astype('uint8')
    nearir = bridge.imgmsg_to_cv2(nearir_in)
    nearir =(nearir/256).astype('uint8')

    merged_4ch=cv2.merge([signal,nearir,reflec,depth])
    merged_3ch=cv2.merge([reflec,reflec,depth])

    image_message = bridge.cv2_to_imgmsg(merged_4ch, "bgra8")  
    image_message.header.stamp = range_in.header.stamp
    image_message.header.frame_id = "ouster_sensor"
    merge_img_pub.publish(image_message)
    
    if (cont%freq==0):            
        if cv2.imwrite(path+"/4_channels/merged_"+str(i+offset)+".png", merged_4ch):
            print("imagen merged_"+str(i)+".png 4 channels Guardada")
            i = i+1
        else:
            print("[ERROR] Imagen 4 channles no guardada. verificar el path: "+path)
        if cv2.imwrite(path+"/3_channels/merged_"+str(j+offset)+".png", merged_3ch):
            print("imagen merged_"+str(j)+".png 3 channels Guardada")
            j = j+1
        else:
            print("[ERROR] Imagen 3 channles no guardada. verificar el path: "+path)

        cont = 0
    cont= cont+1


def main(args=None):
    global bridge, merge_img_pub, cont, i, path, j, offset, freq

    # ------------- CONFIGURATION PARAMETERS ---------------------#
    path = "/home/mice85/aurova-lab/labrobotica/dataset/odometry-kitti/merged_10/"
    offset = 0
    freq = 10
    # -------------------------------------------------------------#

    cont = 0
    i = 0
    j = 0
    try:
        os.mkdir(path+"4_channels")
        os.mkdir(path+"3_channels")
    except:
        print("[ERROR] El path "+path+"no existe, o las carpetas 4 y 3 channels ya han sido creadas anteriormente")

    rclpy.init(args=args)
    node = rclpy.create_node("sync_images")
    bridge = CvBridge()
    range_sub = message_filters.Subscriber(node, Image,'/ouster/range_image')
    signal_sub = message_filters.Subscriber(node, Image,'/ouster/signal_image')
    reflec_sub = message_filters.Subscriber(node, Image,'/ouster/reflec_image')
    nearir_sub = message_filters.Subscriber(node, Image,'/ouster/nearir_image')
    merge_img_pub = node.create_publisher( Image,'/ouster_merged', 10)

    ts = message_filters.TimeSynchronizer([range_sub, signal_sub, reflec_sub, nearir_sub], 10)
    ts.registerCallback(callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

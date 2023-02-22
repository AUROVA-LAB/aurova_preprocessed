#!/usr/bin/env python3

import message_filters
import rospy
import sys
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(range, signal, reflec, nearir):
    global flag, bridge, merge_img_pub
    if flag:
        flag=False
        range = bridge.imgmsg_to_cv2(range)
        range = (range/256).astype('uint8')
        signal = bridge.imgmsg_to_cv2(signal)
        signal =(signal/256).astype('uint8')
        reflec = bridge.imgmsg_to_cv2(reflec)
        reflec =(reflec/256).astype('uint8')
        nearir = bridge.imgmsg_to_cv2(nearir)
        nearir =(nearir/256).astype('uint8')

        merged=cv2.merge([signal,nearir,reflec,range])

        image_message = bridge.cv2_to_imgmsg(merged, "bgra8")
        merge_img_pub.publish(image_message)



if __name__ == '__main__':
    global flag, bridge, merge_img_pub
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

    r = rospy.Rate(20) # 1hz
    while not rospy.is_shutdown():
        flag=True
        r.sleep()
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
    global bridge, merge_img_pub
    depht = bridge.imgmsg_to_cv2(range_in)
    depht = (depht/256).astype('uint8')
    signal = bridge.imgmsg_to_cv2(signal_in)
    signal =(signal/256).astype('uint8')
    reflec = bridge.imgmsg_to_cv2(reflec_in)
    reflec =(reflec/256).astype('uint8')
    nearir = bridge.imgmsg_to_cv2(nearir_in)
    nearir =(nearir/256).astype('uint8')

    merged=cv2.merge([signal,nearir,reflec,depht])
    image_message = bridge.cv2_to_imgmsg(merged, "bgra8")  
    image_message.header.stamp = range_in.header.stamp

    image_message.header.frame_id = "ouster_sensor"
    merge_img_pub.publish(image_message)


def main(args=None):
    global bridge, merge_img_pub
    rclpy.init(args=args)
    node = rclpy.create_node("sync_images")
    flag=False
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

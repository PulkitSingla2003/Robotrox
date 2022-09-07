#!/usr/bin/env python3
#You need to name this node "colour_detector"

from contextlib import nullcontext
from tokenize import String
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def image_callback(img_msg):
    try:
        cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    if(cv_img(20,20) == green):
        pub.publish('Suction Mode')
    elif(cv_img(20,20) == blue):
        pub.publish('Sweep Mode')
    else:
        pub.publish('Not cleaning')
    

rospy.init_node('colour_detector')

green = np.array([0,255,0])
blue = np.array([255,0,0])

bridge = CvBridge()
pub = rospy.Publisher('/cleaning_mode', String)

while not rospy.is_shutdown():
    sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

rospy.spin()
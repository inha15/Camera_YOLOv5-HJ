#!/usr/bin/python3

from E2E_Camera.msg import N_image
import rospy
import numpy as np
import cv2
import time

def imgmsg_to_cv2(img_msg):
    print(img_msg.__dir__())
    im = np.ndarray(shape=(img_msg.height, img_msg.width, img_msg.n_channels),
                    dtype=eval(img_msg.dtype), buffer=img_msg.data)
    return im

def main(data):
    s_time = time.time()
    cv_image = imgmsg_to_cv2(data)
    cv2.imshow("raw_data", cv_image)
    cv2.waitKey(1)

    print(time.time() - s_time)


rospy.init_node("listener", anonymous=True)
rospy.Subscriber("raw_image", N_image, main)
rospy.spin()

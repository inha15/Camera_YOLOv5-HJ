#!/usr/bin/python3

from E2E_Camera.msg import N_image
import os
import time
import PySpin
import numpy as np
import cv2
import rospy
import sys


def cv2_to_n_img(img):
    img_msg = N_image()
    img_msg.height = 1080
    img_msg.width = 1920
    img_msg.n_channels = 3
    img_msg.dtype = "np.uint8"
    img_msg.data = img.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height

    return img_msg

def main():

    # cap = cv2.VideoCapture(3, cv2.CAP_DSHOW)
    cap = cv2.VideoCapture(2)

    pub = rospy.Publisher("raw_image", N_image, queue_size=1)
    rospy.init_node("RealSense", anonymous=True)

    while True:
        if rospy.is_shutdown():
            sys.exit()

        # try:
        _, img = cap.read()
        img = cv2.resize(img, dsize=(1920,1080), interpolation = cv2.INTER_AREA)

        pub.publish(cv2_to_n_img(img))
        # cv2.imshow('11',img)
        # cv2.waitKey(1)
        # except:
        #     print(1)
        #     pass

if __name__ == "__main__":

    main()


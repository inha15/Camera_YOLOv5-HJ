#!/usr/bin/python3

from E2E_Camera.msg import N_image
import numpy as np
import rospy
import cv2
import os

param_list = list(rospy.get_param_names())
for i in param_list:
    tmp_list = i.split("/")
    for j in tmp_list:
        if j == "Type":
            Type = rospy.get_param(i)
        elif j == "Height":
            Height = rospy.get_param(i)
        elif j == "Width":
            Width = rospy.get_param(i)
        elif j == "Path":
            Path = rospy.get_param(i)
        elif j == "FPS":
            FPS = rospy.get_param(i)

Numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U',
                             'int16': '16S', 'int32': '32S', 'float32': '32F',
                             'float64': '64F'}

def cv2_to_n_img(img):
    img_msg = N_image()
    img_msg.height = Height
    img_msg.width = Width
    img_msg.n_channels = 3
    img_msg.dtype = "np.uint8"
    img_msg.data = img.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

pub = rospy.Publisher("raw_image", N_image, queue_size=10)
rospy.init_node("Using_Flie", anonymous=True)
rate = rospy.Rate(FPS)

if Type == "video":
    vid = cv2.VideoCapture(Path)
    while True:
        if rospy.is_shutdown():
            break
        _, img = vid.read()
        pub.publish(cv2_to_n_img(img))
        rate.sleep()

elif Type == "folder":
    img_list = os.listdir(Path)

    while True:
        if rospy.is_shutdown():
            break
        img = cv2.imread(fr"{Path}/{img_list[0]}")
        del img_list[0]
        pub.publish(cv2_to_n_img(img))
        rate.sleep()

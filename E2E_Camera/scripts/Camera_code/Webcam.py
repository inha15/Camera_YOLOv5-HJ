#!/home/autonav-linux/catkin_ws/src/yolov5_ROS/scripts/yolov5/bin/python3

import cv2
import rospy

cap = cv2.VideoCapture(0)

_, frame = cap.read()
print(type(frame))
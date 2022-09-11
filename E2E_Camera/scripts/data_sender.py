#!/usr/bin/python3

import rospy
import numpy as np
from std_msgs.msg import String
from E2E_Camera.msg import camera_data

class Data_sender:


    def __init__(self):
        self.send_txt = ""
        rospy.init_node("data_sender", anonymous=True)
        rospy.Subscriber("/yolov5/yolov5_classes", camera_data, self.read_data)
        self.send_data()
        rospy.spin()

    def read_data(self, data):
        self.send_txt += data.yolov5

    def send_data(self):
        pub = rospy.Publisher("classes", String, queue_size=10)
        rate = rospy.Rate(10)
        while True:
            if rospy.is_shutdown():
                break
            pub.publish(self.send_txt)
            self.send_txt = ""
            rate.sleep()

if __name__ == "__main__":
    Data_sender()
    # while True:
    #     print(rospy.get_published_topics())

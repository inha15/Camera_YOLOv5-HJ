#!/usr/bin/python3

import rospy
from std_msgs.msg import String


def printer(data):
    print(data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/data_sender/classes', String, printer)
    rospy.Subscriber('/data_sender/Filtered_classes', String, printer)
    rospy.spin()

if __name__ == "__main__":
    listener()

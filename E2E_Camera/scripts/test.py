#!/home/autonav-linux/catkin_ws/src/yolov5_ROS/scripts/yolov5/bin/python3


import pyrealsense2 as rs
import numpy as np
import rospy
import cv2

FPS = 30
Width = 1280
Height = 720

Numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U',
                             'int16': '16S', 'int32': '32S', 'float32': '32F',
                             'float64': '64F'}


pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.color, Width, Height, rs.format.bgr8, FPS)

# Start streaming
pipeline.start(config)

while True:
    if rospy.is_shutdown():
        cv2.destroyWindow()
        break
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    color_image = color_image.astype(np.uint8)
    # pub.publish(cv2_to_imgmsg(color_image))
    cv2.imshow("res", color_image)
    cv2.waitKey(1)
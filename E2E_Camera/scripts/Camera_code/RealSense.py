#!/usr/bin/python3

from E2E_Camera.msg import N_image
import pyrealsense2 as rs
import numpy as np
import rospy
import cv2

param_list = list(rospy.get_param_names())
for i in param_list:
    tmp_list = i.split("/")
    for j in tmp_list:
        if j == "FPS":
            FPS = rospy.get_param(i)
        elif j == "Height":
            Height = rospy.get_param(i)
        elif j == "Width":
            Width = rospy.get_param(i)

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
pub = rospy.Publisher("raw_image", N_image, queue_size=10)
rospy.init_node("RealSense", anonymous=True)
if FPS == 0:
    rate = rospy.Rate(60)
else:
    rate = rospy.Rate(FPS)

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
    pub.publish(cv2_to_n_img(color_image))
    rate.sleep()

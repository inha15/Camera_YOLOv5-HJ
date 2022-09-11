#!/usr/bin/python3

from E2E_Camera.msg import N_image
import os
import time
import PySpin
import numpy as np
import cv2
import rospy

param_list = list(rospy.get_param_names())
for i in param_list:
    tmp_list = i.split("/")
    for j in tmp_list:
        if j == "FPS":
            SET_FPS = rospy.get_param(i)
        elif j == "Height":
            IMG_HEIGHT = rospy.get_param(i)
        elif j == "Width":
            IMG_WIDTH = rospy.get_param(i)

Numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U',
                             'int16': '16S', 'int32': '32S', 'float32': '32F',
                             'float64': '64F'}

def cv2_to_n_img(img):
    img_msg = N_image()
    img_msg.height = IMG_HEIGHT
    img_msg.width = IMG_WIDTH
    img_msg.n_channels = 3
    img_msg.dtype = "np.uint8"
    img_msg.data = img.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

def configure_custom_image_settings(nodemap):
    """
    카메라 초기 설정.
    :param nodemap: Camera info
    :return:
    """
    print('*** CONFIGURING CUSTOM IMAGE SETTINGS ***')
    try:
        result = True
        node_pixel_format = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))

        if PySpin.IsAvailable(node_pixel_format) and PySpin.IsWritable(node_pixel_format):
            # node_pixel_format_mono8 = PySpin.CEnumEntryPtr(node_pixel_format.GetEntryByName('RGB8Packed'))
            node_pixel_format_mono8 = PySpin.CEnumEntryPtr(node_pixel_format.GetEntryByName('RGB8Packed'))
            if PySpin.IsAvailable(node_pixel_format_mono8) and PySpin.IsReadable(node_pixel_format_mono8):
                pixel_format_mono8 = node_pixel_format_mono8.GetValue()
                node_pixel_format.SetIntValue(pixel_format_mono8)
                # node_pixel_format.SetIntValue(100)
                print('Pixel format set to %s...' % node_pixel_format.GetCurrentEntry().GetSymbolic())
            else:
                print('Pixel format mono 8 not available...')
        else:
            print('Pixel format not available...')

        node_offset_x = PySpin.CIntegerPtr(nodemap.GetNode('OffsetX'))

        if PySpin.IsAvailable(node_offset_x) and PySpin.IsWritable(node_offset_x):
            node_offset_x.SetValue(node_offset_x.GetMin())
            print('Offset X set to %i...' % node_offset_x.GetMin())
        else:
            print('Offset X not available...')

        node_offset_y = PySpin.CIntegerPtr(nodemap.GetNode('OffsetY'))

        if PySpin.IsAvailable(node_offset_y) and PySpin.IsWritable(node_offset_y):
            node_offset_y.SetValue(node_offset_y.GetMin())
            print('Offset Y set to %i...' % node_offset_y.GetMin())

        else:
            print('Offset Y not available...')

        node_width = PySpin.CIntegerPtr(nodemap.GetNode('Width'))

        if PySpin.IsAvailable(node_width) and PySpin.IsWritable(node_width):
            width_to_set = node_width.GetMax()
            node_width.SetValue(IMG_WIDTH)
            print('Width set to %i...' % node_width.GetValue())
        else:
            print('Width not available...')

        node_height = PySpin.CIntegerPtr(nodemap.GetNode('Height'))
        if PySpin.IsAvailable(node_height) and PySpin.IsWritable(node_height):
            height_to_set = node_height.GetMax()
            node_height.SetValue(IMG_HEIGHT)
            print('Height set to %i...' % node_height.GetValue())
        else:
            print('Height not available...')
        if not bool(SET_FPS):
            node_FPSAuto = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionFrameRateAuto"))
            node_FPSAutoOff = PySpin.CEnumEntryPtr(node_FPSAuto.GetEntryByName("Off"))
            node_FPSAuto.SetIntValue(node_FPSAutoOff.GetValue())

            node_FPSEnable = PySpin.CBooleanPtr(nodemap.GetNode("AcquisitionFrameRateEnable"))
            if PySpin.IsAvailable(node_FPSEnable) and PySpin.IsWritable(node_FPSEnable):
                node_FPSEnable.SetValue(True)
                print('Set FPS True')
            else:
                print("false to change")

            node_FPSrate = PySpin.CFloatPtr(nodemap.GetNode("AcquisitionFrameRate"))
            if PySpin.IsAvailable(node_FPSrate) or PySpin.IsWritable(node_FPSrate):
                node_FPSrate.SetValue(SET_FPS)
                print('change FPS rate')
            else:
                print('false')


    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False
    return result

def acquire_images(cam, nodemap, nodemap_tldevice):
    """
    다음 이미지를 불러옴
    :param cam:
    :param nodemap:
    :param nodemap_tldevice:
    :param q:
    :return:
    """
    try:
        result = True
        node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
        if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
            print('Unable to set acquisition mode to continuous (enum retrieval). Aborting...')
            return False

        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
        if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(
                node_acquisition_mode_continuous):
            print('Unable to set acquisition mode to continuous (entry retrieval). Aborting...')
            return False

        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()
        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)
        cam.BeginAcquisition()
        pub = rospy.Publisher("raw_image", N_image, queue_size=1)
        rospy.init_node("RealSense", anonymous=True)
        if SET_FPS == 0:
            rate = rospy.Rate(60)
        else:
            rate = rospy.Rate(SET_FPS)

        cnt = 0
        while True:

            if rospy.is_shutdown():
                break

            try:
                s_time = time.time()
                image_result = cam.GetNextImage() # 0.056 / 0.07
                image_result.buffer = None # almost 0.00001
                image_converted = image_result.Convert(PySpin.PixelFormat_BGR8, PySpin.HQ_LINEAR) # 0.003 / 0.07
                img = image_converted.GetNDArray() # 0.002 / 0.07
                pub.publish(cv2_to_n_img(img))

                cnt += 1
                image_result.Release() # almost 0.00001
                rate.sleep()
                print("Camera : %f" % (1/(time.time()-s_time)))
            except PySpin.SpinnakerException as ex:
                print('Error: %s' % ex)
                return False
        cam.EndAcquisition()

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False
    return result


def run_single_camera(cam):
    try:

        result = True
        nodemap_tldevice = cam.GetTLDeviceNodeMap()
        cam.Init()
        nodemap = cam.GetNodeMap()
        configure_custom_image_settings(nodemap)
        result &= acquire_images(cam, nodemap, nodemap_tldevice)
        cam.DeInit()
    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False
    return result


def main():

    try:
        test_file = open('test.txt', 'w+')
    except IOError:
        print('Unable to write to current directory. Please check permissions.')
        input('Press Enter to exit...')
        return False

    test_file.close()
    os.remove(test_file.name)
    t = time.strftime('%Y_%m_%d/%H_%M', time.localtime(time.time()))
    path = "./save/%s" % t
    if not os.path.isdir(path):
        os.makedirs(path)
    result = True

    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    num_cameras = cam_list.GetSize()

    if num_cameras == 0:
        cam_list.Clear()
        system.ReleaseInstance()
        return False

    for i, cam in enumerate(cam_list):
        print('+++++++++++++++++++++++++++Running+++++++++++++++++++++++++++++++')
        s_node_map = cam.GetTLStreamNodeMap()
        handling_mode = PySpin.CEnumerationPtr(s_node_map.GetNode('StreamBufferHandlingMode'))
        handling_mode_entry = PySpin.CEnumEntryPtr(handling_mode.GetCurrentEntry())
        handling_mode_entry = handling_mode.GetEntryByName('NewestOnly')
        handling_mode.SetIntValue(handling_mode_entry.GetValue())
        result &= run_single_camera(cam)
        print('---------------------------complete-------------------------------')
    del cam
    cam_list.Clear()
    system.ReleaseInstance()

if __name__ == "__main__":

    main()

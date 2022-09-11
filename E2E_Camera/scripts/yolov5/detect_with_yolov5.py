#!/usr/bin/python3

import numpy as np
import copy
import time

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from E2E_Camera.msg import N_image, camera_data

import argparse
import os
import sys
from pathlib import Path
from cv_bridge import CvBridge

import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.augmentations import letterbox
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync

try:
    param_list = list(rospy.get_param_names())
    for i in param_list:
        tmp_list = i.split("/")
        for j in tmp_list:
            if j == "PT":
                PT_FILE = rospy.get_param(i)

    PT_FILE = PT_FILE.split(",")
    PT_FILE = [f'{ROOT / str(i)}' for i in PT_FILE]
except:
    PT_FILE = "yolov5s.pt"

class YOLOv5:


    def __init__(self):
        self.img = False
        self.IMGSZ = [0, 0]
        rospy.init_node("yolov5-main", anonymous=True)
        rospy.Subscriber("raw_image", N_image, self.read_data)
        self.pub = rospy.Publisher("yolov5_classes", camera_data, queue_size=10)
        self.pub1 = rospy.Publisher("img", Image, queue_size=10)
        self.rate = rospy.Rate(50)
        self.run(weights=PT_FILE)

    def read_data(self, img_msg):
        self.img = np.ndarray(shape=(img_msg.height, img_msg.width, img_msg.n_channels),
                              dtype=eval(img_msg.dtype), buffer=img_msg.data)
        self.IMGSZ = [img_msg.width, img_msg.height]

    def make_im0(self, img, stride=32):

        img0 = copy.deepcopy(img)
        img_size = self.IMGSZ[0]
        img = letterbox(img0, img_size, stride=stride, auto=True)[0]

        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img)

        return img, img0

    @torch.no_grad()
    def run(self, weights=ROOT / 'yolov5s.pt',
            conf_thres=0.6,
            iou_thres=0.45,
            max_det=1000,
            device="",
            line_thickness=3,
            hide_labels=False,
            hide_conf=False
            ):
        print(weights)
        while type(self.img) == bool: pass

        device = select_device(device)
        model = DetectMultiBackend(weights, device=device, dnn=False, fp16=False)
        stride, names, pt = model.stride, model.names, model.pt
        imgsz = check_img_size(tuple(self.IMGSZ), s=stride)

        half = False
        half &= pt and device != "cpu"
        if pt:
            model.model.half() if half else model.model.float()

        cudnn.benchmark = True
        bs = 1

        model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))
        dt, seen = [0.0, 0.0, 0.0], 0

        while True:

            s_time = time.time()
            if rospy.is_shutdown():
                cv2.destroyAllWindows()
                break

            im, im0 = self.make_im0(self.img, stride)
            bridge = CvBridge()
            self.pub1.publish(bridge.cv2_to_imgmsg(im0, encoding="passthrough"))
            t1 = time_sync()
            im = torch.from_numpy(im).to(device)
            im = im.half() if model.fp16 else im.float()
            im /= 255
            if len(im.shape) == 3:
                im = im[None]
            t2 = time_sync()
            dt[0] += t2 - t1

            pred = model(im, augment=False, visualize=False)
            t3 = time_sync()
            dt[1] += t3 - t2

            pred = non_max_suppression(pred, conf_thres, iou_thres, None, False, max_det=max_det)
            dt[2] += time_sync() - t3

            save_txt = ""
            for i, det in enumerate(pred):

                seen += 1
                annotator = Annotator(im0, line_width=line_thickness, example=str(names))

                if len(det):

                    det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                    for c in det[:, -1].unique():
                        n = (det[:, -1 == c]).sum()

                    save_txt = ""

                    for *xyxy, conf, cls in reversed(det):
                        c = int(cls)
                        label = None if hide_labels else (names[c] if hide_conf else f"{names[c]} {conf:.2f}")
                        annotator.box_label(xyxy, label, color=colors(c, True))
                        save_list = [str(i.tolist()) for i in xyxy]
                        pre_txt = ", ".join(save_list)
                        if str(names[c]) in ["SUV", "SD"]:
                            save_txt += f"CAR-{pre_txt}-{conf:.2f}/"
                        else:
                            save_txt += f"{names[c]}-{pre_txt}-{conf:.2f}/"

                    im0 = annotator.result()
                cv2.imshow("res", im0)
                cv2.waitKey(1)
                camera_msg = camera_data()
                camera_msg.yolov5 = save_txt

                self.pub.publish(camera_msg)
                self.rate.sleep()

            # print("time : ", time.time()-s_time)
        rospy.spin()
if __name__ == "__main__":
    YOLOv5()
#!/usr/bin/env python
#coding=utf-8

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import _init_paths

import rospy
from bit_vision_msgs.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import os

from opts import opts
from detectors.detector_factory import detector_factory

import numpy as np
from progress.bar import Bar
import time
import torch
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

try:
  from external.nms import soft_nms
except:
  print('NMS not imported! If you need it,'
        ' do \n cd $CenterNet_ROOT/src/lib/external \n make')
from models.decode import ctdet_decode
from models.utils import flip_tensor
from utils.image import get_affine_transform
from utils.post_process import ctdet_post_process
from utils.debugger import Debugger

from lib.detectors.base_detector import BaseDetector

image_ext = ['jpg', 'jpeg', 'png', 'webp']
video_ext = ['mp4', 'mov', 'avi', 'mkv']
time_stats = ['tot', 'load', 'pre', 'net', 'dec', 'post', 'merge']

#检测是否成功标志
Detection_Flag = False
TL_X = 0
TL_Y = 0
BR_X = 0
BR_Y = 0


class L_Detector(BaseDetector):
    def __init__(self, opt):
        super(L_Detector, self).__init__(opt)

    def process(self, images, return_time=False):
        with torch.no_grad():
            output = self.model(images)[-1]
            hm = output['hm'].sigmoid_()
            wh = output['wh']
            reg = output['reg'] if self.opt.reg_offset else None
            if self.opt.flip_test:
                hm = (hm[0:1] + flip_tensor(hm[1:2])) / 2
                wh = (wh[0:1] + flip_tensor(wh[1:2])) / 2
                reg = reg[0:1] if reg is not None else None
            torch.cuda.synchronize()
            forward_time = time.time()
            dets = ctdet_decode(hm, wh, reg=reg, cat_spec_wh=self.opt.cat_spec_wh, K=self.opt.K)

        if return_time:
            return output, dets, forward_time
        else:
            return output, dets

    def post_process(self, dets, meta, scale=1):
        dets = dets.detach().cpu().numpy()
        dets = dets.reshape(1, -1, dets.shape[2])
        dets = ctdet_post_process(
            dets.copy(), [meta['c']], [meta['s']],
            meta['out_height'], meta['out_width'], self.opt.num_classes)
        for j in range(1, self.num_classes + 1):
            dets[0][j] = np.array(dets[0][j], dtype=np.float32).reshape(-1, 5)
            dets[0][j][:, :4] /= scale
        return dets[0]

    def merge_outputs(self, detections):
        results = {}
        for j in range(1, self.num_classes + 1):
            results[j] = np.concatenate(
                [detection[j] for detection in detections], axis=0).astype(np.float32)
            if len(self.scales) > 1 or self.opt.nms:
                soft_nms(results[j], Nt=0.5, method=2)
        scores = np.hstack(
            [results[j][:, 4] for j in range(1, self.num_classes + 1)])
        if len(scores) > self.max_per_image:
            kth = len(scores) - self.max_per_image
            thresh = np.partition(scores, kth)[kth]
            for j in range(1, self.num_classes + 1):
                keep_inds = (results[j][:, 4] >= thresh)
                results[j] = results[j][keep_inds]
        return results

    def debug(self, debugger, images, dets, output, scale=1):
        detection = dets.detach().cpu().numpy().copy()
        detection[:, :, :4] *= self.opt.down_ratio
        for i in range(1):
            img = images[i].detach().cpu().numpy().transpose(1, 2, 0)
            img = ((img * self.std + self.mean) * 255).astype(np.uint8)
            pred = debugger.gen_colormap(output['hm'][i].detach().cpu().numpy())
            debugger.add_blend_img(img, pred, 'pred_hm_{:.1f}'.format(scale))
            debugger.add_img(img, img_id='out_pred_{:.1f}'.format(scale))
            for k in range(len(dets[i])):
                if detection[i, k, 4] > self.opt.center_thresh:
                    debugger.add_coco_bbox(detection[i, k, :4], detection[i, k, -1],
                                           detection[i, k, 4],
                                           img_id='out_pred_{:.1f}'.format(scale))

    def show_results(self, debugger, image, results):
        debugger.add_img(image, img_id='ctdet')
        for j in range(1, self.num_classes + 1):
            for bbox in results[j]:
                if bbox[4] > self.opt.vis_thresh:
                    debugger.add_coco_bbox(bbox[:4], j - 1, bbox[4], img_id='ctdet')
        print('Follow the code in show all imgs')
        debugger.show_all_imgs(pause=self.pause)

    def generate_results(self, debugger, image, results):
        debugger.add_img(image, img_id='ctdet')
        self.opt.vis_thresh = 0.2
        for j in range(1, self.num_classes + 1):
            count = 0
            for bbox in results[j]:
                if bbox[4] > self.opt.vis_thresh:
                    count = count + 1
                    TL_X = bbox[0]
                    TL_Y = bbox[1]
                    BR_X = bbox[2]
                    BR_Y = bbox[3]

                    print('bbox is : {:.1f},{:.1f},{:.1f},{:.1f}'.format(TL_X, TL_Y, BR_X, BR_Y))
                    Detection_Flag = True
            if(count == 0):
                Detection_Flag = False
            else:
                Detection_Flag = True
                    
ImageL = np.zeros((2209,1243,3))
#定义回调函数
def callback(imgmsg):
    global im, flag_im
    bridge = CvBridge()
    im = bridge.imgmsg_to_cv2(imgmsg, 'bgr8')
    Image = im
    flag_im = True               

def demo(opt,Image):
    os.environ['CUDA_VISIBLE_DEVICES'] = opt.gpus_str
    opt.debug = max(opt.debug, 1)

    Detector = L_Detector
    detector =Detector(opt)
    ret = detector.run(Image)

    time_str = ''
    for stat in time_stats:
        time_str = time_str + '{} {:.3f}s |'.format(stat, ret[stat])
        print(time_str)


def get_detection_result(req):
    
    demo(opt,ImageL)

    res = L_detection_srvResponse()
    res.success_flag = Detection_Flag
    res.x1 = TL_X
    res.y1 = TL_Y
    res.x2 = BR_X
    res.y2 = BR_Y
    return res
    
def add_two_ints_server():
    rospy.init_node('Shelf_Detection')
    rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, callback)
    s = rospy.Service('Get_L_Loc', L_detection_srv, get_detection_result)
    rospy.loginfo("Ready to handle the request:")
    rospy.spin()

if __name__ == '__main__':
    opt = opts().init()
    add_two_ints_server()

    

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
import torch
from opts import opts
from detectors.detector_factory import detector_factory

import numpy as np
from progress.bar import Bar
import time
import torch


image_ext = ['jpg', 'jpeg', 'png', 'webp']
video_ext = ['mp4', 'mov', 'avi', 'mkv']
time_stats = ['tot', 'load', 'pre', 'net', 'dec', 'post', 'merge']

try:
    from external.nms import soft_nms_39
except:
    print('NMS not imported! If you need it,'
          ' do \n cd $CenterNet_ROOT/src/lib/external \n make')
from models.decode import kps_decode
from models.utils import flip_tensor, flip_lr_off, flip_lr
from utils.image import get_affine_transform
from utils.post_process import kps_post_process
from utils.debugger import Debugger

from detectors.base_detector import BaseDetector

import sys
# if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
#     sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

#检测是否成功标志
Detection_Flag = False
# bbox_x1 = 0
# bbox_y1 = 0
# bbox_x2 = 0
# bbox_y2 = 0
kps_x_list = [0,0,0,0,0,0]
kps_y_list = [0,0,0,0,0,0]

class KPSDetector(BaseDetector):
    def __init__(self, opt):
        super(KPSDetector, self).__init__(opt)

    def process(self, images, return_time=False):
        with torch.no_grad():
            torch.cuda.synchronize()
            output = self.model(images)[-1]
            output['hm'] = output['hm'].sigmoid_()
            if self.opt.hm_hp and not self.opt.mse_loss:
                output['hm_hp'] = output['hm_hp'].sigmoid_()

            reg = output['reg'] if self.opt.reg_offset else None
            hm_hp = output['hm_hp'] if self.opt.hm_hp else None
            hp_offset = output['hp_offset'] if self.opt.reg_hp_offset else None
            torch.cuda.synchronize()
            forward_time = time.time()

            if self.opt.flip_test:
                output['hm'] = (output['hm'][0:1] + flip_tensor(output['hm'][1:2])) / 2
                output['wh'] = (output['wh'][0:1] + flip_tensor(output['wh'][1:2])) / 2
                output['hps'] = (output['hps'][0:1] +
                                 flip_lr_off(output['hps'][1:2], self.flip_idx)) / 2
                hm_hp = (hm_hp[0:1] + flip_lr(hm_hp[1:2], self.flip_idx)) / 2 \
                    if hm_hp is not None else None
                reg = reg[0:1] if reg is not None else None
                hp_offset = hp_offset[0:1] if hp_offset is not None else None

            dets = kps_decode(
                output['hm'], output['wh'], output['hps'],
                reg=reg, hm_hp=hm_hp, hp_offset=hp_offset, K=self.opt.K)

        if return_time:
            return output, dets, forward_time
        else:
            return output, dets

    def post_process(self, dets, meta, scale=1):
        dets = dets.detach().cpu().numpy().reshape(1, -1, dets.shape[2])
        dets = kps_post_process(
            dets.copy(), [meta['c']], [meta['s']],
            meta['out_height'], meta['out_width'])
        for j in range(1, self.num_classes + 1):
            dets[0][j] = np.array(dets[0][j], dtype=np.float32).reshape(-1, 17)
            # import pdb; pdb.set_trace()
            dets[0][j][:, :4] /= scale
            dets[0][j][:, 5:] /= scale
        return dets[0]

    def merge_outputs(self, detections):
        results = {}
        results[1] = np.concatenate(
            [detection[1] for detection in detections], axis=0).astype(np.float32)
        if self.opt.nms or len(self.opt.test_scales) > 1:
            soft_nms_39(results[1], Nt=0.5, method=2)
        results[1] = results[1].tolist()
        return results

    def debug(self, debugger, images, dets, output, scale=1):
        dets = dets.detach().cpu().numpy().copy()
        dets[:, :, :4] *= self.opt.down_ratio
        dets[:, :, 5:39] *= self.opt.down_ratio
        img = images[0].detach().cpu().numpy().transpose(1, 2, 0)
        img = np.clip(((
                               img * self.std + self.mean) * 255.), 0, 255).astype(np.uint8)
        pred = debugger.gen_colormap(output['hm'][0].detach().cpu().numpy())
        debugger.add_blend_img(img, pred, 'pred_hm')
        if self.opt.hm_hp:
            pred = debugger.gen_colormap_hp(
                output['hm_hp'][0].detach().cpu().numpy())
            debugger.add_blend_img(img, pred, 'pred_hmhp')

    def show_results(self, debugger, image, results):
        debugger.add_img(image, img_id='multi_pose')
        for bbox in results[1]:
            if bbox[4] > self.opt.vis_thresh:
                debugger.add_coco_bbox(bbox[:4], 0, bbox[4], img_id='multi_pose')
                debugger.add_KPS_hp(bbox[5:17], img_id='multi_pose')
        debugger.show_all_imgs(pause=self.pause)

    def generate_results(self, debugger, image, results):
        debugger.add_img(image, img_id='ctdet')
        global Detection_Flag
        global kps_x_list
        global kps_y_list
        self.opt.vis_thresh = 0.5
        for j in range(1, self.num_classes + 1):
            for bbox in results[j]:
                if bbox[4] > self.opt.vis_thresh:
                    Detection_Flag = True
                    kps_x_list = bbox[5:11]
                    kps_y_list = bbox[11:17]
                    print (kps_x_list)
                    print (kps_y_list)


ImageL = np.zeros((2209,1243,3))
#定义回调函数
def callback(imgmsg):
    global im, flag_im
    bridge = CvBridge()
    im = bridge.imgmsg_to_cv2(imgmsg, 'bgr8')
    ImageL = im
    flag_im = True  


def demo(opt,Image):
    os.environ['CUDA_VISIBLE_DEVICES'] = opt.gpus_str
    opt.debug = max(opt.debug, 1)
    Detector = KPSDetector
    detector = Detector(opt)

    ret = detector.run(Image)
    time_str = ''
    for stat in time_stats:
        time_str = time_str + '{} {:.3f}s |'.format(stat, ret[stat])
        print(time_str)


def get_detection_result(req):

    demo(opt, ImageL)
    res = L_KPS_srvResponse()
    global Detection_Flag
    res.success_flag = Detection_Flag
    res.X_list = kps_x_list
    res.Y_list = kps_y_list

    Detection_Flag = False  # 复位标识量
    return res

if __name__ == '__main__':
    opt = opts().init()
    rospy.init_node('Shelf_KPS_Detection')
    rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, callback) #/zed/zed_node/left/image_rect_color    /CameraMER/ContinuousImage
    s = rospy.Service('Get_L_KPS', L_KPS_srv, get_detection_result)
    rospy.loginfo("Ready to handle the request:")

    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        rate.sleep()
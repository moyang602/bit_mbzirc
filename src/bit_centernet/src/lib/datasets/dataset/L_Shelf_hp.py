from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import pycocotools.coco as coco
from pycocotools.cocoeval import COCOeval
import numpy as np
import json
import os

import torch.utils.data as data


class L_Shelf_HP(data.Dataset):
    num_classes = 1
    num_kps = 6                        #对于L架是6个
    default_resolution = [1280, 2240]  #先将这里改成resize之后的分辨率
    mean = np.array([0.32429235, 0.32851489, 0.33174885],
                    dtype=np.float32).reshape(1, 1, 3)
    std = np.array([0.20327683, 0.21368938, 0.20512214],
                   dtype=np.float32).reshape(1, 1, 3)
    # flip_idx = [[1, 2], [3, 4], [5, 6]]

    def __init__(self, opt, split):
        super(L_Shelf_HP, self).__init__()
        self.edges = [[0,1],[1,2],[2,3],[3.4],[4,5]]
        self.acc_idxs = [1,2,3,4,5,6]
        #数据集的根目录
        self.data_dir = '/media/srt/dataset/L_Shelf_0114'  #/media/srt/resource/Halcon_Project/L_shelf_dataset/HG_Dataset
        self.img_dir = os.path.join(self.data_dir, 'JPEGimages')  #keypoint_img
        if split == 'val':
            self.annot_path = os.path.join(
                self.data_dir, 'json_2020_0117',
                'val_L.json')
        else:
            #给关键点检测任务赋予新的任务名 KPS
            if opt.task == 'KPS':
                self.annot_path = os.path.join(
                    self.data_dir, 'json_2020_0117',
                    'train_L.json') #annotations
            if split == 'test':
                self.annot_path = os.path.join(
                    self.data_dir, 'json_2020_0117',
                    'test_L.json').format(split)
            else:
                self.annot_path = os.path.join(
                    self.data_dir, 'json_2020_0117', 'train_L.json')
        self.max_objs = 2
        self.class_name = [
            '__background__', 'L']
        self._valid_ids = [1]
        self.cat_ids = {v: i for i, v in enumerate(self._valid_ids)}
        self.voc_color = [(v // 32 * 64 + 64, (v // 8) % 4 * 64, v % 8 * 32) \
                          for v in range(1, self.num_classes + 1)]
        self._data_rng = np.random.RandomState(123)
        self._eig_val = np.array([0.2141788, 0.01817699, 0.00341571],
                                 dtype=np.float32)
        self._eig_vec = np.array([
            [-0.58752847, -0.69563484, 0.41340352],
            [-0.5832747, 0.00994535, -0.81221408],
            [-0.56089297, 0.71832671, 0.41158938]
        ], dtype=np.float32)
        # self.mean = np.array([0.485, 0.456, 0.406], np.float32).reshape(1, 1, 3)
        # self.std = np.array([0.229, 0.224, 0.225], np.float32).reshape(1, 1, 3)

        self.split = split
        self.opt = opt

        print('==> initializing L shelf {} data.'.format(split))
        self.coco = coco.COCO(self.annot_path)
        self.images = self.coco.getImgIds()
        self.num_samples = len(self.images)

        print('Loaded {} {} samples'.format(split, self.num_samples))

    def _to_float(self, x):
        return float("{:.2f}".format(x))

    def convert_eval_format(self, all_bboxes):
        # import pdb; pdb.set_trace()
        # print('when it is eval will use the function?')
        detections = []
        for image_id in all_bboxes:
            for cls_ind in all_bboxes[image_id]:
                category_id = 1                     #self._valid_ids[cls_ind - 1]
                for dets in all_bboxes[image_id][cls_ind]:
                    bbox = dets[:4]
                    bbox[2] -= bbox[0]
                    bbox[3] -= bbox[1]
                    score = dets[4]
                    bbox_out = list(map(self._to_float, bbox))
                    keypoints = np.concatenate([
                        np.array(dets[5:17], dtype=np.float32).reshape(-1, 2),
                        np.ones((6, 1), dtype=np.float32)], axis=1).reshape(18).tolist()
                    keypoints = list(map(self._to_float, keypoints))

                    detection = {
                        "image_id": int(image_id),
                        "category_id": int(category_id),
                        "bbox": bbox_out,
                        "score": float("{:.2f}".format(score)),
                        "keypoints": keypoints
                    }
                    # if len(bbox) > 5:
                    #     extreme_points = list(map(self._to_float, bbox[5:13]))
                    #     detection["extreme_points"] = extreme_points
                    detections.append(detection)
        return detections

    def __len__(self):
        return self.num_samples

    def save_results(self, results, save_dir):
        json.dump(self.convert_eval_format(results),
                  open('{}/results.json'.format(save_dir), 'w'))

    def run_eval(self, results, save_dir):

        self.save_results(results, save_dir)
        print('save_dir is : ',save_dir)
        coco_dets = self.coco.loadRes('{}/results.json'.format(save_dir))
        coco_eval = COCOeval(self.coco, coco_dets, "keypoints")
        coco_eval.evaluate()
        coco_eval.accumulate()
        coco_eval.summarize()
        coco_eval = COCOeval(self.coco, coco_dets, "bbox")
        coco_eval.evaluate()
        coco_eval.accumulate()
        coco_eval.summarize()

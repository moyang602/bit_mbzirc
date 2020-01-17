from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import torch.utils.data as data
import numpy as np
import torch
import json
import cv2
import os
from utils.image import flip, color_aug
from utils.image import get_affine_transform, affine_transform
from utils.image import gaussian_radius, draw_umich_gaussian, draw_msra_gaussian
from utils.image import draw_dense_reg
import math
import matplotlib

try:
    import ipdb
except:
    import pdb as ipdb


class KPSDataset(data.Dataset):
    def _coco_box_to_bbox(self, box):
        bbox = np.array([box[0], box[1], box[0] + box[2], box[1] + box[3]],
                        dtype=np.float32)
        return bbox

    def _get_border(self, border, size):
        i = 1
        while size - border // i <= border // i:
            i *= 2
        return border // i

    def __getitem__(self, index):
        img_id = self.images[index]
        file_name = self.coco.loadImgs(ids=[img_id])[0]['file_name']
        img_path = os.path.join(self.img_dir, file_name)
        ann_ids = self.coco.getAnnIds(imgIds=[img_id])
        anns = self.coco.loadAnns(ids=ann_ids)
        num_objs = min(len(anns), self.max_objs)

        img = cv2.imread(img_path)
        #根据图像文件名,读入对应的关键点标注文件
        (filepath, tempfilename) = os.path.split(img_path)
        (filename, extension) = os.path.splitext(tempfilename)
        kps_path = os.path.join('/media/srt/dataset/L_Shelf_0114/Kps_Ann',
                                filename + '_kps.npy')  #/media/srt/resource/Halcon_Project/L_shelf_dataset/HG_Dataset/kps
        kps_raw = np.load(kps_path)
        c3 = np.ones(6)
        kps_ann=np.column_stack((kps_raw,c3))  #将关键点维度变为[6,3]

        height, width = img.shape[0], img.shape[1]
        c = np.array([img.shape[1] / 2., img.shape[0] / 2.], dtype=np.float32)
        s = max(img.shape[0], img.shape[1]) * 1.0  #对crop,shift进行赋值
        input_h, input_w = self.opt.input_h, self.opt.input_w  #在opt中定义的分辨率
        rot = 0

        flipped = False
        if self.split == 'train':
            if not self.opt.not_rand_crop:
                s = s * np.random.choice(np.arange(0.6, 1.4, 0.1))
                w_border = self._get_border(128, img.shape[1])
                h_border = self._get_border(128, img.shape[0])
                c[0] = np.random.randint(low=w_border, high=img.shape[1] - w_border)
                c[1] = np.random.randint(low=h_border, high=img.shape[0] - h_border)
            else:
                sf = self.opt.scale     #0
                cf = self.opt.shift      #0
                c[0] += s * np.clip(np.random.randn() * cf, -2 * cf, 2 * cf)
                c[1] += s * np.clip(np.random.randn() * cf, -2 * cf, 2 * cf)
                s = s * np.clip(np.random.randn() * sf + 1, 1 - sf, 1 + sf)
            #加上multi-pose中的随机旋转
            if np.random.random() < self.opt.aug_rot:
                rf = self.opt.rotate
                rot = np.clip(np.random.randn() * rf, -rf * 2, rf * 2)

            # if np.random.random() < self.opt.flip:
            #     flipped = True
            #     img = img[:, ::-1, :]
            #     c[0] = width - c[0] - 1

        # 对输入执行仿射变换
        trans_input = get_affine_transform(
            c, s,rot, [input_w, input_h])
        inp = cv2.warpAffine(img, trans_input,
                             (input_w, input_h),
                             flags=cv2.INTER_LINEAR)
        inp = (inp.astype(np.float32) / 255.)
        if self.split == 'train' and not self.opt.no_color_aug:
            color_aug(self._data_rng, inp, self._eig_val, self._eig_vec)
        inp = (inp - self.mean) / self.std
        inp = inp.transpose(2, 0, 1)
        test_image = inp[1]   #用于与kps_hp可视化使用

        output_h = input_h // self.opt.down_ratio
        output_w = input_w // self.opt.down_ratio
        num_classes = self.num_classes
        num_kps = 6  #点数是否需要+1 ?

        trans_output = get_affine_transform(c, s, 0, [output_w, output_h])
        trans_output_rot =get_affine_transform(c, s, rot, [output_w, output_h])

        hm = np.zeros((num_classes, output_h, output_w), dtype=np.float32) #中心对应的hp
        hm_hp = np.zeros((num_kps, output_h, output_w), dtype=np.float32)  #kps对应的hp
        #此处只是初始化,未赋值
        dense_kps = np.zeros((num_kps, 2, output_h, output_w),
                             dtype=np.float32)
        dense_kps_mask = np.zeros((num_kps, output_h, output_w),
                                  dtype=np.float32)
        wh = np.zeros((self.max_objs, 2), dtype=np.float32)
        dense_wh = np.zeros((2, output_h, output_w), dtype=np.float32)
        kps= np.zeros((num_kps, num_kps * 2), dtype=np.float32)  #其他关键点指向某个关键点的向量
        reg = np.zeros((self.max_objs, 2), dtype=np.float32)
        ind = np.zeros((self.max_objs), dtype=np.int64)
        reg_mask = np.zeros((self.max_objs), dtype=np.uint8)
        kps_mask = np.zeros((self.max_objs, self.num_kps * 2), dtype=np.uint8)
        hp_offset = np.zeros((self.max_objs * num_kps, 2), dtype=np.float32)
        hp_ind = np.zeros((self.max_objs * num_kps), dtype=np.int64)
        hp_mask = np.zeros((self.max_objs * num_kps), dtype=np.int64)
        cat_spec_wh = np.zeros((self.max_objs, num_classes * 2), dtype=np.float32)
        cat_spec_mask = np.zeros((self.max_objs, num_classes * 2), dtype=np.uint8)

        draw_gaussian = draw_msra_gaussian if self.opt.mse_loss else \
            draw_umich_gaussian
        #获取标注各项数据的标志
        gt_det = []
        for k in range(num_objs):
            ann = anns[k]
            bbox = self._coco_box_to_bbox(ann['bbox'])
            cls_id = int(self.cat_ids[ann['category_id']])
            #pts的读入方式可以自行定义
            pts = np.array(kps_ann, np.float32).reshape(num_kps, 3)  #原来的按照coco数据集json标注读入

            bbox[:2] = affine_transform(bbox[:2], trans_output)
            bbox[2:] = affine_transform(bbox[2:], trans_output)
            bbox[[0, 2]] = np.clip(bbox[[0, 2]], 0, output_w - 1)
            bbox[[1, 3]] = np.clip(bbox[[1, 3]], 0, output_h - 1)
            h, w = bbox[3] - bbox[1], bbox[2] - bbox[0]
            if h > 0 and w > 0:
                radius = gaussian_radius((math.ceil(h), math.ceil(w)))
                radius = max(0, int(radius))
                radius = self.opt.hm_gauss if self.opt.mse_loss else radius
                ct = np.array(
                    [(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2], dtype=np.float32)
                ct_int = ct.astype(np.int32)
                wh[k] = 1. * w, 1. * h
                ind[k] = ct_int[1] * output_w + ct_int[0]
                reg[k] = ct - ct_int
                reg_mask[k] = 1
                num_kpts = pts[:, 2].sum()
                if num_kpts == 0:
                    hm[cls_id, ct_int[1], ct_int[0]] = 0.9999
                    reg_mask[k] = 0
                cat_spec_wh[k, cls_id * 2: cls_id * 2 + 2] = wh[k]
                cat_spec_mask[k, cls_id * 2: cls_id * 2 + 2] = 1

                hp_radius = gaussian_radius((math.ceil(h), math.ceil(w)))
                hp_radius = self.opt.hm_gauss \
                    if self.opt.mse_loss else max(0, int(hp_radius))
                for j in range(num_kps):
                    if pts[j, 2] > 0:
                        #如果关键点的第3位>0,则对关键点进行变换
                        pts[j, :2] = affine_transform(pts[j, :2], trans_output)   #对关键点进行变换
                        if pts[j, 0] >= 0 and pts[j, 0] < output_w and \
                                pts[j, 1] >= 0 and pts[j, 1] < output_h:
                            #计算其他点指向该点的向量
                            kps[j, j * 2: j * 2 + 2] = pts[:, :2] - pts[j, :2]
                            kps_mask[k, j * 2: j * 2 + 2] = 1
                            pt_int = pts[j, :2].astype(np.int32)
                            hp_offset[k * num_kps + j] = pts[j, :2] - pt_int
                            hp_mask[k * num_kps + j] = 1
                            if self.opt.dense_hp:
                                #必须在中心点hm gassian之前画
                                print('draw dense hp!!!')
                                draw_dense_reg(dense_kps[j], hm[cls_id], ct_int,
                                               pts[j, :2] - ct_int, radius, is_offset=True)
                                draw_gaussian(dense_kps_mask[j], ct_int, radius)
                            draw_gaussian(hm_hp[j], pt_int, hp_radius)
                            heatmap = np.squeeze(hm_hp[j])                      #(1,160,240)
                            heatmap = cv2.resize(heatmap, (960, 640), interpolation=cv2.INTER_CUBIC)
                            new_image = test_image + heatmap * 2
                            array_name = 'forbidden_s_c_kps_hp/visual_kps_' + str(index) +'_'+str(j)+ '.png'
                            # matplotlib.image.imsave(array_name, new_image)
                #画中心点的高斯图
                draw_gaussian(hm[cls_id], ct_int, radius)
                heatmap = np.squeeze(hm[cls_id])  # (1,160,240)
                heatmap = cv2.resize(heatmap, (960, 640), interpolation=cv2.INTER_CUBIC)
                new_image = test_image + heatmap * 2
                array_name = 'visual_center_' + str(index) + '.png'
                # matplotlib.image.imsave(array_name, new_image)


                if self.opt.dense_wh:
                    draw_dense_reg(dense_wh, hm.max(axis=0), ct_int, wh[k], radius)
                gt_det.append([ct[0] - w / 2, ct[1] - h / 2,
                               ct[0] + w / 2, ct[1] + h / 2, 1] +
                              pts[:, :2].reshape(num_kps * 2).tolist() + [cls_id])
                # gt_det.append([ct[0] - w / 2, ct[1] - h / 2,
                #                ct[0] + w / 2, ct[1] + h / 2, 1, cls_id])
        #在原来的基础上增加了 'hps','hps_mask'
        ret = {'input': inp, 'hm': hm, 'reg_mask': reg_mask, 'ind': ind, 'wh': wh,
               'hps': kps, 'hps_mask': kps_mask}
        if self.opt.dense_hp:
            dense_kps = dense_kps.reshape(num_kps * 2, output_h, output_w)
            dense_kps_mask = dense_kps_mask.reshape(
                num_kps, 1, output_h, output_w)
            dense_kps_mask = np.concatenate([dense_kps_mask, dense_kps_mask], axis=1)
            dense_kps_mask = dense_kps_mask.reshape(
                num_kps * 2, output_h, output_w)
            ret.update({'dense_hps': dense_kps, 'dense_hps_mask': dense_kps_mask})
            del ret['hps'], ret['hps_mask']
        if self.opt.dense_wh:
            hm_a = hm.max(axis=0, keepdims=True)
            dense_wh_mask = np.concatenate([hm_a, hm_a], axis=0)
            ret.update({'dense_wh': dense_wh, 'dense_wh_mask': dense_wh_mask})
            del ret['wh']
        elif self.opt.cat_spec_wh:
            ret.update({'cat_spec_wh': cat_spec_wh, 'cat_spec_mask': cat_spec_mask})
            del ret['wh']
        if self.opt.reg_offset:
            ret.update({'reg': reg})
        if self.opt.hm_hp:
            ret.update({'hm_hp': hm_hp})
        if self.opt.reg_hp_offset:
            ret.update({'hp_offset': hp_offset, 'hp_ind': hp_ind, 'hp_mask': hp_mask})
        if self.opt.debug > 0 or not self.split == 'train':
            gt_det = np.array(gt_det, dtype=np.float32) if len(gt_det) > 0 else \
                np.zeros((1, 6), dtype=np.float32)
            meta = {'c': c, 's': s, 'gt_det': gt_det, 'img_id': img_id}
            ret['meta'] = meta
        return ret
# coding:utf-8
# 运行前请先做以下工作：
# pip install lxml
# 将所有的图片及xml文件存放到xml_dir指定的文件夹下，并将此文件夹放置到当前目录下
#

import os
import glob
import json
import shutil
import numpy as np
import xml.etree.ElementTree as ET

START_BOUNDING_BOX_ID = 1
save_path = "."


def get(root, name):
    return root.findall(name)


def get_and_check(root, name, length):
    vars = get(root, name)
    if len(vars) == 0:
        raise NotImplementedError('Can not find %s in %s.' % (name, root.tag))
    if length and len(vars) != length:
        raise NotImplementedError('The size of %s is supposed to be %d, but is %d.' % (name, length, len(vars)))
    if length == 1:
        vars = vars[0]
    return vars


def convert(xml_list, json_file):
    json_dict = {"images": [], "type": "instances", "annotations": [], "categories": []}
    categories = pre_define_categories.copy()
    bnd_id = START_BOUNDING_BOX_ID
    all_categories = {}
    for index, line in enumerate(xml_list):
        # print("Processing %s"%(line))
        xml_f = line
        tree = ET.parse(xml_f)
        root = tree.getroot()

        filename = os.path.basename(xml_f)[:-4] + ".png"
        image_id = 20190000001 + index
        size = get_and_check(root, 'size', 1)
        width = int(get_and_check(size, 'width', 1).text)
        height = int(get_and_check(size, 'height', 1).text)
        image = {'file_name': filename, 'height': height, 'width': width, 'id': image_id}
        json_dict['images'].append(image)
        #  Currently we do not support segmentation
        segmented = get_and_check(root, 'segmented', 1).text
        assert segmented == '0'
        for obj in get(root, 'object'):
            category = get_and_check(obj, 'name', 1).text
            if category in all_categories:
                all_categories[category] += 1
            else:
                all_categories[category] = 1
            if category not in categories:
                if only_care_pre_define_categories:
                    continue
                new_id = len(categories) + 1
                print(
                    "[warning] category '{}' not in 'pre_define_categories'({}), create new id: {} automatically".format(
                        category, pre_define_categories, new_id))
                categories[category] = new_id
            category_id = categories[category]
            bndbox = get_and_check(obj, 'bndbox', 1)
            xmin = int(float(get_and_check(bndbox, 'xmin', 1).text))
            ymin = int(float(get_and_check(bndbox, 'ymin', 1).text))
            xmax = int(float(get_and_check(bndbox, 'xmax', 1).text))
            ymax = int(float(get_and_check(bndbox, 'ymax', 1).text))
            assert (xmax > xmin), "xmax <= xmin, {}".format(line)
            assert (ymax > ymin), "ymax <= ymin, {}".format(line)
            o_width = abs(xmax - xmin)
            o_height = abs(ymax - ymin)
            ann = {'area': o_width * o_height, 'iscrowd': 0, 'image_id':
                image_id, 'bbox': [xmin, ymin, o_width, o_height],
                   'category_id': category_id, 'id': bnd_id, 'ignore': 0,
                   'segmentation': []}
            json_dict['annotations'].append(ann)
            bnd_id = bnd_id + 1

    for cate, cid in categories.items():
        cat = {'supercategory': 'L', 'id': cid, 'name': cate}
        json_dict['categories'].append(cat)
    json_fp = open(json_file, 'w')
    json_str = json.dumps(json_dict)
    json_fp.write(json_str)
    json_fp.close()
    print("------------create {} done--------------".format(json_file))
    print("find {} categories: {} -->>> your pre_define_categories {}: {}".format(len(all_categories),
                                                                                  all_categories.keys(),
                                                                                  len(pre_define_categories),
                                                                                  pre_define_categories.keys()))
    print("category: id --> {}".format(categories))
    print(categories.keys())
    print(categories.values())


if __name__ == '__main__':
    # 定义你自己的类别
    classes = ['L']
    pre_define_categories = {}
    for i, cls in enumerate(classes):
        pre_define_categories[cls] = i + 1
    # 这里也可以自定义类别id，把上面的注释掉换成下面这行即可
    # pre_define_categories = {'a1': 1, 'a3': 2, 'a6': 3, 'a9': 4, "a10": 5}
    only_care_pre_define_categories = True  # or False

    # 保存的json文件
    save_json_train = '/media/srt/dataset/L_Shelf_0103/train_L.json'
    save_json_val = '/media/srt/dataset/L_Shelf_0103/val_L.json'
    save_json_test = '/media/srt/dataset/L_Shelf_0103/test_L.json'

    # 初始文件所在的路径
    xml_dir = "/media/srt/dataset/L_Shelf_0103/xml"
    xml_list = glob.glob(xml_dir + "/*.xml")
    xml_list = np.sort(xml_list)

    # 打乱数据集
    np.random.seed(10)
    np.random.shuffle(xml_list)

    # 按比例划分打乱后的数据集
    train_ratio = 0.85
    val_ratio = 0.1
    train_num = int(len(xml_list) * train_ratio)
    val_num = int(len(xml_list) * val_ratio)
    xml_list_train = xml_list[:train_num]
    xml_list_val = xml_list[train_num: train_num+val_num]
    xml_list_test = xml_list[train_num+val_num:]

    # 将xml文件转为coco文件，在指定目录下生成三个json文件（train/test/food）
    convert(xml_list_train, save_json_train)
    convert(xml_list_val, save_json_val)
    convert(xml_list_test, save_json_test)

    print("-" * 50)
    print("train number:", len(xml_list_train))
    print("val number:", len(xml_list_val))
    print("test number:", len(xml_list_val))

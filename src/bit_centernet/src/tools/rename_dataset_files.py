import numpy as np
import os


def generate_list(path):
	files =  os.listdir(path)
	imgs = []
	for fi in files:
		(filename, extension) = os.path.splitext(fi)
		if(extension == ".png"):
			imgs.append(fi)
	return imgs

img_root = '/media/srt/resource/Halcon_Project/L_shelf_dataset/HG_Dataset/keypoint_img'  #图像路径
new_img_root = '/media/srt/resource/Halcon_Project/L_shelf_dataset/HG_Dataset/20200103/imgs'
kps_label_root = '/media/srt/resource/Halcon_Project/L_shelf_dataset/HG_Dataset/kps'         #关键点路径
new_kps_root = '/media/srt/resource/Halcon_Project/L_shelf_dataset/HG_Dataset/20200103/kps_ann'
xml_label_root = '/media/srt/resource/Halcon_Project/L_shelf_dataset/HG_Dataset/bbox_xml'    #包围框标注
new_xml_root = '/media/srt/resource/Halcon_Project/L_shelf_dataset/HG_Dataset/20200103/xml'
for imgname in generate_list(img_root):
    (filename, extension) = os.path.splitext(imgname)
    src_img= os.path.join(img_root,imgname)
    dst_img = os.path.join(new_img_root, '2019_' + filename+'.png')
    os.rename(src_img, dst_img)
    print('converting %s to %s ...' % (src_img, dst_img))

    src_kps = os.path.join(kps_label_root,filename+'_kps'+'.npy')
    dst_kps = os.path.join(new_kps_root,'2019_' + filename+'_kps'+'.npy')
    os.rename(src_kps, dst_kps)
    print('converting %s to %s ...' % (src_kps, dst_kps))

    src_xml = os.path.join(xml_label_root,filename+'.xml')
    dst_xml = os.path.join(new_xml_root,'2019_' + filename+'.xml')
    os.rename(src_xml, dst_xml)
    print('converting %s to %s ...' % (src_xml, dst_xml))


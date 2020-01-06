import numpy as np
import os

#生成路径下所有图像文件名组成的list
def generate_list(path):
	files =  os.listdir(path)
	imgs = []
	for fi in files:
		(filename, extension) = os.path.splitext(fi)
		if(extension == ".png"):
			imgs.append(fi)
	return imgs

img_root = '/media/srt/resource/Halcon_Project/L_shelf_dataset/L_shelf_2020'
label_root = '/media/srt/resource/Halcon_Project/L_shelf_dataset/txts_2020'
for imgname in generate_list(img_root):
    (filename, extension) = os.path.splitext(imgname)
    ann_row_root = os.path.join(label_root,filename+'_row.txt')
    ann_col_root = os.path.join(label_root,filename+'_col.txt')
    kps_label=[]
    rowfile = open(ann_row_root, 'r')
    rowlist = rowfile.readlines()
    colfile = open(ann_col_root, 'r')
    collist = colfile.readlines()
    for index in range(2, 8):
        coor = []
        rowline = rowlist[index].strip('\n')
        colline = collist[index].strip('\n')
        coor_row = rowline.split(' ', 1)[1]
        coor_col = colline.split(' ', 1)[1]
        coor.append(float(coor_col))
        coor.append(float(coor_row))
        kps_label.append(coor)

    kps_name = os.path.join('/media/srt/resource/Halcon_Project/L_shelf_dataset/kps_2020',filename+'_kps')
    kps = np.array(kps_label)
    np.save(kps_name, kps)



'''
file_path = "/media/srt/resource/Halcon_Project/L_shelf_dataset/HG_Dataset/keypoint_img/halconL2.png"
(filepath,tempfilename) = os.path.split(file_path)
(filename,extension) = os.path.splitext(tempfilename)
print(filename)
'''
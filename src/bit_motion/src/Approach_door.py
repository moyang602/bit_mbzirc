#!/usr/bin/env python
#coding:utf-8

import rospy, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from bit_vision_msgs.srv import *


import skimage.transform as st
from skimage import io
import numpy as np
import matplotlib.pyplot as plt
import tf
import os

# import sys
# sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")

global InRangeDistance,LineLikeThreshold,DistanceBTCarlink2Brick,needTolerance,ThinkLineLength,Map_size,resolution

Map_size = 10                               # 激光雷达画的图的大小，单位：米
resolution = 0.05                           # 地图分辨率，单位：米
InRangeDistance = 8.0                       # 激光雷达可以接手的距离
LineLikeThreshold = 0.1                     # 在遍历直线时认为是直线的阈值
DistanceBTCarlink2Brick = 1.0 + LineLikeThreshold       # 0.628为激光雷达到car_link的距离
needTolerance = 0.1                         # 认为是需要的线段长度的阈值，+-，单位：米 
ThinkLineLength = 0.3                       # 认为霍夫变换的结果是一条直线的最小长度，单位：米

def scan_callback(msg):

    global ranges,image,h,theta,d,imagesize,delta_angle,min_angle,horizion_num
    
    ranges = np.array(msg.ranges) * 0.9998476951563913
    min_angle = msg.angle_min
    max_angle = msg.angle_max
    delta_angle = msg.angle_increment
    horizion_num = len(ranges)

    # Map_size = np.max(ranges[np.logical_not(np.isinf(ranges))])
    
    imagesize = int(round(Map_size/resolution))
    image = np.zeros((2*imagesize, 2*imagesize))  #背景图为一片黑

    # 画图
    for i in range( 0, len(ranges) ):
        x = ranges[i] * math.cos( min_angle + i * delta_angle )
        y = -ranges[i] * math.sin( min_angle + i * delta_angle )
        if not np.isinf(x) and (not np.isinf(y)):
            if math.fabs(x) < Map_size - 0.1 and math.fabs(y) < Map_size - 0.1 :
                image[-int(round(x/resolution)) + imagesize ,-int(round(y/resolution)) + imagesize] = 255



    # 进行hough线变换
    h, theta, d = st.hough_line(image)

    # hough概率变换
    # lines = st.probabilistic_hough_line(image, threshold=10, line_length=40,line_gap=20)

num = 0
def LaserProcHandle(req):
    res = LaserProcResponse()
    pos = Pose()

    if req.BrickType == "D":
        need= 2.0
        pass
    else:
        rospy.logwarn("LaserProc ERROR: WRONG BRICK TYPE")
        res.VisionData.Flag = 0
        res.VisionData.header.stamp = rospy.Time.now()
        return res

    try:
        if dist_center > InRangeDistance:
            res.VisionData.Flag = 0
            res.VisionData.header.stamp = rospy.Time.now()
        else:
            if FindReady == 1:
                rospy.logwarn(len(leng))
                res.VisionData.Flag = 1
                res.VisionData.header.stamp = rospy.Time.now()
                res.VisionData.header.frame_id = "velodyne"
                resultok = 0
                for l in range(len(leng)):
                    if math.fabs(leng[l] - need) < needTolerance:
                        rospy.loginfo("Found need line: %f" % need)    
                        rospy.logwarn([len(leng),l,len(xy_M)]) 
                        targetx = use_xy_M[l][0]*resolution - imagesize*resolution - DistanceBTCarlink2Brick * math.sin(-use_turnangle) 
                        targety = use_xy_M[l][1]*resolution - imagesize*resolution + DistanceBTCarlink2Brick * math.cos(-use_turnangle) 
                        
                        pos.position.x = -targety
                        pos.position.y = targetx
                        ori = tf.transformations.quaternion_from_euler(0,0,-turnangle)
                        pos.orientation.x = ori[0]
                        pos.orientation.y = ori[1]
                        pos.orientation.z = ori[2]
                        pos.orientation.w = ori[3]

                        os.system('gnome-screenshot -f /home/ugvcontrol/image/Laser/' + str(rospy.Time.now().to_sec())+ '.png')
                        # 保存图片
                        # global num
                        # save = np.array(image,dtype=np.uint8)
                        # io.imsave('/home/ugvcontrol/image/Laser/my_map' + str(num) + '.png', save)     # 修改为合适的路径
                        # num += 1
                        res.VisionData.Pose = pos
                        resultok = 1
                        break
                if resultok ==0:
                    res.VisionData.Flag = 0
                    res.VisionData.header.stamp = rospy.Time.now()

            else:
                res.VisionData.Flag = 0
                res.VisionData.header.stamp = rospy.Time.now()
    except Exception as e:
        print(e)
        res.VisionData.Flag = 0
        res.VisionData.header.stamp = rospy.Time.now()
    finally:
        return res
                

if __name__ == '__main__':

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1) # to move the robot
    pub_middle = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=1)
    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)   # to read the laser scanner
    rospy.init_node('approach_door')

    s = rospy.Service('LaserProc_Door', LaserProc, LaserProcHandle)
    rate = rospy.Rate(10)

    image = np.zeros((100,100))

    fig=plt.figure()
    ax2=fig.add_subplot(1,1,1)
    plt.ion()
    print("OK")

    global dist_center
    global leng
    global turnangle
    global xy_M
    global FindReady

    while not rospy.is_shutdown():
        plt.cla()

        #显示检测出的线条
        ax2.imshow(image, plt.cm.gray)
        row1, col1 = image.shape
        ax2.axis((0, col1, row1, 0))
        ax2.set_title('Detected lines')

        for a, angle, dist in zip(*st.hough_line_peaks(h, theta, d ,num_peaks=np.inf)):      # TODO improve the robustness
            y0 = (dist - 0 * np.cos(angle)) / np.sin(angle)
            y1 = (dist - col1 * np.cos(angle)) / np.sin(angle)
            ax2.plot((0, col1), (y0, y1), '-b', alpha = 0.5)

            # 计算直线到小车激光雷达的距离
            dist_center = math.fabs(- col1 *(imagesize - y0) - (y0 - y1) * imagesize) / math.sqrt( col1**2 + (y0 - y1)**2 ) * resolution 

            # 得到转动角度
            if angle > 0:
                turnangle = pi/2 - angle
            else:
                turnangle = -pi/2 - angle

            # 得到直线的（起始角度、终止角度）在激光雷达坐标系下的标号
            startAngle = int(round(math.atan2(imagesize , imagesize - y0 ) /delta_angle))
            if (y0 + y1) > 2*imagesize:        # 在身后
                turnangle += pi
                endAngle = int(round(2*pi - math.atan2(imagesize , imagesize - y1)  /delta_angle))
                dir = -1
            else :
                endAngle = int(round(- math.atan2(imagesize , imagesize - y1) /delta_angle))
                dir = 1

            startAngle = int(round(horizion_num/2)) - startAngle
            endAngle = -endAngle + dir*int(round(horizion_num/2))       # 在背后时加360

            # 转动角度限幅
            if turnangle > pi:
                turnangle -= 2*pi
            elif turnangle < -pi:
                turnangle += 2*pi

            # 开始直线遍历，阈值为LineLikeThreshold
            if a* resolution > ThinkLineLength:
                
                startpoint = startAngle
                endpoint = startAngle
                segment = []
                lastDist = False
                
                # 遍历直线上的激光雷达数据
                for i in range( startAngle , endAngle, dir):
                    if math.fabs(turnangle) > 80*pi/180.0:
                        continue
                    # 得到距离差
                    delta_dist= math.fabs(ranges[i % horizion_num] * math.cos(pi -  i*delta_angle - turnangle)) - dist_center

                    # 进行二值化
                    d_dist = math.fabs(delta_dist) < LineLikeThreshold
                    
                    # 检测上升沿和下降沿，上升沿开始，下降沿停止，取出一段直线，存在segment中
                    if d_dist == False:
                        if lastDist == True:
                            startpoint = i
                    if d_dist == True:
                        if lastDist == False:
                            endpoint = lastI 
                            segment.append([startpoint,endpoint])
                    lastDist = d_dist
                    lastI = i

            leng = []
            xy_M = []
            try:        # 如果有结果的话
                for seg in segment:
                    d0 = ranges[seg[0] % horizion_num] * math.sin(pi - seg[0]* delta_angle - turnangle)
                    d1 = ranges[seg[1] % horizion_num] * math.sin(pi - seg[1]* delta_angle - turnangle)
                    leng.append(math.fabs(d0 - d1))

                    x_0 = -ranges[seg[0] % horizion_num] *math.sin(seg[0]* delta_angle) /resolution + imagesize
                    y_0 = ranges[seg[0] % horizion_num] *math.cos(seg[0]* delta_angle) /resolution + imagesize
                    x_1 = -ranges[seg[1] % horizion_num] *math.sin(seg[1]* delta_angle) /resolution + imagesize
                    y_1 = ranges[seg[1] % horizion_num] *math.cos(seg[1]* delta_angle) /resolution + imagesize
                    
                    xy_M.append([(x_0+x_1)/2,(y_0+ y_1)/2])
                    ax2.plot((x_0, x_1),(y_0, y_1))
                
            except:
                print("no result in Line")

            # 遍历所有找到的直线段的长度，从中点的记录中计算目标
            for l in range(len(leng)):
                if math.fabs(leng[l] - 2.0) < needTolerance:
                    use_xy_M = xy_M
                    use_turnangle = turnangle
                    FindReady = 1
                    break
                else:
                    FindReady = 0
                    continue
                    
        plt.show()
        plt.pause(0.1)

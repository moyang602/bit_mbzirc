#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
This program is debugged by Harden Qiu
"""

import roslib
import rospy
import sensor_msgs
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

import sys, select, termios, tty, os, time
import cv2
import numpy as np
import tf     #tf 里面有定义订阅者的函数


#肤色识别
def skin(frame):
    lower = np.array([0, 40, 80], dtype="uint8")
    upper = np.array([20, 255, 255], dtype="uint8")
    converted = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    skinMask = cv2.inRange(converted, lower, upper)
    skinMask = cv2.GaussianBlur(skinMask, (5, 5), 0)
    skin = cv2.bitwise_and(frame, frame, mask=skinMask)
    return skin


#求最大连通域的中心点坐标
def centroid(max_contour):
    moment = cv2.moments(max_contour)
    if moment['m00'] != 0:
        cx = int(moment['m10'] / moment['m00'])
        cy = int(moment['m01'] / moment['m00'])
        return cx, cy
    else:
        return None

minSepration = 0.1      # meters
flex_separate =1.618
interesting_area = 2


def callback(data):
    # print(data.info.origin.position , len(data.data))
    # global map_count
    # if map_count > 5.1:
    #     map_count -= 5
    global flex_separate
    global trans

    frame = []
    real = []
    resolution = data.info.resolution
    origin = data.info.origin.position
    for i in range(data.info.height ):
        tempframe = []
        realframe = []
        for j in range(data.info.width):
            temp_map = data.data[ i*data.info.width + data.info.width - j - 1]
            realframe.append(temp_map)
            if temp_map < 0:
                tempframe.append(0)
            elif temp_map < 50:
                tempframe.append(125)
            else:
                tempframe.append(225)
        frame.append(tempframe)
        real.append(realframe)
    frame = np.array(frame, dtype=np.uint8)
    real = np.array(real, dtype=np.uint8)
    print(frame.shape)
    

    # ret, frame = cv2.threshold(frame, 50, 255, cv2.THRESH_BINARY)
    _,contours, he = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE ) 
    print("number of contours:%d" % len(contours))
    cv2.drawContours(real, contours, -1, (50, 50, 50), 3)

    
    cv2.imshow('Original',real) 
    ''' 基于轮廓的探索  '''
    last = this = 0
    temp = []
    posible_set = []
    i = j = 0
    backfind = 1
    finish = 1
    while j < len(contours):
        if len(contours[j]) >= 5:
            i = 0
            len_contour = len(contours[j])
            end = len_contour
            while i < end:      # 循环所有的轮廓点，寻找其中可走的连接段
                this = frame[contours[j][i % len_contour][0][1]][contours[j][i % len_contour][0][0]]
                # print(this)
                if this <= 127 + 50:      # 可走的段
                    finish = 0      # 如果有一个可以走的点就认为没有完成
                    if i <= 0 and backfind == 1:
                        i -= 1
                        end -= 1
                        if end == 0:
                            # 四周皆空，请随意
                            break
                        continue

                    temp.append(list(contours[j][i % len_contour][0]))

                else:
                    backfind = 0
                    if last > 127 + 50:
                        if temp:
                            posible_set.append(temp)
                            temp = []
                last = this
                i += 1
        j += 1
    print('avilable exit number：%d' % len(posible_set))
    result_score = []
    result_goal = []
    goal = [0] * 2
    if finish == 0:     # 轮廓上有未建图的

        if posible_set :        # 如果有一条未建图的连通曲线
            i =0
            while i < len(posible_set):      # 遍历每一条，得到每一条的分数
                this_len= len(posible_set[i])
                canGoThrough = minSepration < resolution * np.linalg.norm(np.array(posible_set[i][0]) - np.array(posible_set[i][-1]))
                # print(canGoThrough)
                thisDistance = np.linalg.norm(resolution * np.array(posible_set[i][int(this_len/flex_separate)]) - np.array([trans[1],trans[0]]))
                if canGoThrough == 1:
                    result_score.append(thisDistance*(thisDistance-interesting_area)/this_len**2)     # 可以走的解的评估，越小越好
                    result_goal.append(posible_set[i][int(this_len/flex_separate)]) 
                i += 1
            
            # print(result_score)
            if result_score :       # 有可供车辆通过的连通曲线， 寻找其中评价函数最低的
                # print(result_score)
                goal = result_goal[result_score.index(np.min(result_score))]
            else:        # 如果都没分， 全都小于最小值， 无处可走， 认为完成
                finish = 1
        else:
            # 四周皆空, 生成向前的目标点
            goal[0] = int(trans[0]/resolution) + contours[0][0][0][0]
            goal[1] = int(trans[1]/resolution) + contours[0][0][0][1]

        if success_cnt > 2:    # 停了太久
            flex_separate += 2

            
    print('success_cnt %d,%f'%(success_cnt,flex_separate))
    print(trans)
    cv2.circle(frame, tuple([data.info.width - int((trans[0]- origin.x)/resolution),  int((trans[1]- origin.y)/resolution )]), 0, 255, 4)
    cv2.circle(frame, tuple(goal), 0, 255, 4)
    print(finish, goal)
    goal_pos = PoseStamped()
    goal_pos.header.frame_id = "map"
    goal_pos.header.stamp = rospy.Time.now()
    goal_pos.pose.position.x = (data.info.width - goal[0]) * resolution + origin.x
    goal_pos.pose.position.y = goal[1] * resolution + origin.y
    goal_pos.pose.position.z = 0
    goal_pos.pose.orientation.w = 1

    pos.publish(goal_pos)

    #处理后显示
    cv2.imshow("Live",frame)

    cv2.waitKey(950)    

def resultCB(data):
    global success_cnt
    global last_success
    global flex_separate
    if data.status.status == 3:
        success_cnt += 1
        if last_success != 3:
            flex_separate = 1.618
    else :
        success_cnt = 0
    last_success = success_cnt

if __name__ == '__main__':

    # pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    # pub_ee = rospy.Publisher('endeffCmd',EndEffector,queue_size=1)

    global trans
    success_cnt = 0
    last_success = 0

    rospy.init_node('map_dealer')
    listener = tf.TransformListener()  #定义listener  一旦定义listener  ，他就开始接受信息，并且可以缓冲10S.
    
    pos = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size= 1)
    rospy.Subscriber('move_base/result', MoveBaseActionResult, resultCB)

    rospy.Rate(100)
    rospy.Subscriber("map", OccupancyGrid, callback)
    while not rospy.is_shutdown():
        try: 
             trans, _ = listener.lookupTransform('/car_link', '/map', rospy.Time(0))        # 可以缓冲10S内最近的信息。
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
            continue
        # print(trans)
    cv2.destroyAllWindows()


#!/usr/bin/env python3
#coding:utf-8

import rospy, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi
import math
from geometry_msgs.msg import PoseStamped


import skimage.transform as st
from skimage import io
import numpy as np
import matplotlib.pyplot as plt
import tf

import sys
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")

dist = []
def scan_callback(msg):
    global range_front
    global range_right
    global range_left
    global ranges
    global min_front,i_front, min_right,i_right, min_left ,i_left
    global i_range
    
    global resolution,image,h,theta,d,imagesize,delta_angle,min_angle,horizion_num
    resolution = 0.05
    
    ranges = np.array(msg.ranges)
    min_angle = msg.angle_min
    max_angle = msg.angle_max
    delta_angle = msg.angle_increment
    horizion_num = len(ranges)


    # max_size = np.max(ranges[np.logical_not(np.isinf(ranges))])
    max_size = 10
    imagesize = int(round(max_size/resolution))

    image = np.zeros((2*imagesize, 2*imagesize))  #背景图

    for i in range( 0, len(ranges) ):
        x = ranges[i] * math.cos( min_angle + i * delta_angle )
        y = -ranges[i] * math.sin( min_angle + i * delta_angle )
        if not np.isinf(x) and (not np.isinf(y)):
            if math.fabs(x) < max_size - 0.1 and math.fabs(y) < max_size - 0.1 :
                image[-int(round(x/resolution)) + imagesize ,-int(round(y/resolution)) + imagesize] = 255

    save = np.array(image,dtype=np.uint8)
    io.imsave('./my_map.png', save)
    # hough线变换
    h, theta, d = st.hough_line(image)

    # lines = st.probabilistic_hough_line(image, threshold=10, line_length=40,line_gap=20)

if __name__ == '__main__':

    # Create the node
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1) # to move the robot
    pub_middle = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=1)
    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)   # to read the laser scanner
    rospy.init_node('maze_explorer')


    middlepoint=PoseStamped()
    rate = rospy.Rate(10)

    print("in")

    image = np.zeros((100,100))

    fig=plt.figure()
    # ax0=fig.add_subplot(1,3,1)
    # ax1=fig.add_subplot(1,3,2)
    ax2=fig.add_subplot(1,1,1)
    # ax3=fig.add_subplot(2,2,4)
    plt.ion()
    print("OK")

    while not rospy.is_shutdown():


        plt.cla()
        #显示原始图片
        # ax0.imshow(image, plt.cm.gray)
        # ax0.set_title('Image')
        # ax0.set_axis_off()

        # #显示hough变换所得数据
        # ax1.imshow(np.log(1 + h))
        # ax1.set_title('Hough transform')
        # ax1.set_xlabel('Angles (degrees)')
        # ax1.set_ylabel('Distance (pixels)')
        # ax1.axis('image')
        # ax1.set_axis_off()

        #显示检测出的线条
        ax2.imshow(image, plt.cm.gray)
        row1, col1 = image.shape
        for a, angle, dist in zip(*st.hough_line_peaks(h, theta, d, num_peaks=1)):
            y0 = (dist - 0 * np.cos(angle)) / np.sin(angle)
            y1 = (dist - col1 * np.cos(angle)) / np.sin(angle)
            ax2.plot((0, col1), (y0, y1), '-b', alpha = 0.5)
            # print(a*resolution, angle,dist)
        ax2.axis((0, col1, row1, 0))
        ax2.set_title('Detected lines')
    
        if angle > 0:
            turnangle = pi/2 - angle
        else:
            turnangle = -pi/2 - angle

        startAngle = int(round(math.atan2(imagesize , imagesize - y0 ) /delta_angle))
        
        if (y0 + y1) > 2*imagesize:        # 在身后
            turnangle += pi
            endAngle = int(round(2*pi - math.atan2(imagesize , imagesize - y1)  /delta_angle))
            dir = -1
        else :
            endAngle = int(round(- math.atan2(imagesize , imagesize - y1) /delta_angle))
            dir = 1

        if turnangle > pi:
            turnangle -= 2*pi
        elif turnangle < -pi:
            turnangle += 2*pi

        dist_center = math.fabs(- col1 *(imagesize - y0) - (y0 - y1) * imagesize) / math.sqrt( col1**2 + (y0 - y1)**2 ) * resolution

        # print(startAngle,endAngle,a*resolution,turnangle,dist_center)
        startAngle = int(round(horizion_num/2)) - startAngle
        endAngle = -endAngle + dir*int(round(horizion_num/2))       # 在背后时加360


        threshold = 0.07
        if a* resolution > 0.3:
            startpoint = startAngle
            endpoint = startAngle
            segment = []
            issegment = False
            delta_dist = []
            lastDist = False
            # print (ranges)
            # print(startAngle, endAngle + (dir +1 )*180, dir)
            for i in range( startAngle , endAngle, dir):   

                delta_dist= math.fabs(ranges[i % horizion_num] * math.cos(pi -  i*delta_angle - turnangle)) - dist_center
                # print(delta_dist)
                d_dist = math.fabs(delta_dist) < threshold

                if d_dist == True:
                    if lastDist == False:
                        startpoint = i
                if d_dist == False:
                    if lastDist == True:
                        endpoint = lastI 
                        segment.append([startpoint,endpoint])
                lastDist = d_dist
                lastI = i

        try:
            # print(startAngle, endAngle, segment)
            leng = []
            xy_M = []
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
            print("no result")

        # print(startAngle,endAngle,segment)
        # print(leng)

        need = 1.2

        dis = 0.6 + 0.628

        last_targetx = 0
        last_targety = 0
        for l in range(len(leng)):
            # print(leng[l])
            if math.fabs(leng[l] - need) < 0.2:
                print("Found need line:%f"%need)
                targetx = xy_M[l][0]*resolution - imagesize*resolution - dis * math.sin(-turnangle) 
                targety = xy_M[l][1]*resolution - imagesize*resolution + dis * math.cos(-turnangle) 

                # print(targetx,targety,turnangle)
                middlepoint.header.frame_id="velodyne"
                middlepoint.header.stamp = rospy.Time.now()
                middlepoint.pose.position.x = -targety
                middlepoint.pose.position.y = targetx
                ori = tf.transformations.quaternion_from_euler(0,0,-turnangle)
                middlepoint.pose.orientation.x = ori[0]
                middlepoint.pose.orientation.y = ori[1]
                middlepoint.pose.orientation.z = ori[2]
                middlepoint.pose.orientation.w = ori[3]
                
                if math.fabs(last_targetx - targetx) > 0.1 or math.fabs(last_targety - targety) > 0.1:
                    pub_middle.publish(middlepoint)
                    print("pubilished")
              
                last_targetx = targetx
                last_targety = targety

        plt.show()
        plt.pause(0.1)

        # plt.cla()
        # ax3.imshow(image * 0)
        # for line in lines:
        #     p0, p1 = line
        #     ax3.plot((p0[0], p1[0]), (p0[1], p1[1]))
        #     print(line)
        # print("once")
        # row2, col2 = image.shape
        # ax3.axis((0, col2, row2, 0))
        # ax3.set_title('Probabilistic Hough')
        # ax3.set_axis_off()
       
        


        


    # while not rospy.is_shutdown():    
    #     command = Twist()
    #     middlepoint=PoseStamped()

        # if(not turingcomplete):
        #     Turning(i_range,angle_speed)     
        # if ((ranges[0]>=set_dis)&(turingcomplete)):
        #     Approching(set_dis,linear_speed)                           
        # elif((not stoponce)&approchingcomplete):
        #     Stopatpoint()
        # # if((min_front<=set_dis)&(not movingcomplete)):
        # #     print("The length of brick:")
        # #     print(Movingtomiddle())
        # #     middlepoint.header.frame_id="base_link"
        # #     middlepoint.header.stamp = rospy.Time.now()
        # #     middlepoint.pose.position.x =0
        # #     middlepoint.pose.position.y =Movingtomiddle()   
        # #     middlepoint.pose.orientation.w=1
        # #     pub_middle.publish(middlepoint)

        # if(stoponce&(not movingcomplete)):        
        #     print("The length of brick:")
        #     print(Movingtomiddle())
        #     middlepoint.header.frame_id="base_link"
        #     middlepoint.header.stamp = rospy.Time.now()
        #     middlepoint.pose.position.x =0
        #     middlepoint.pose.position.y =Movingtomiddle()   
        #     middlepoint.pose.orientation.w=1
        #     pub_middle.publish(middlepoint)

            #  pulish_move  

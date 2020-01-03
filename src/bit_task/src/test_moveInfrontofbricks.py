#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import tf 
from bit_task_msgs.msg import BrickInfo
import numpy as np
import rospy 
import geometry_msgs.msg as gmsg
from visualization_msgs.msg import MarkerArray,Marker



tft = tf.TransformerROS()


def makeLmarker(tf_OrignOnMap):

    tx = tft.fromTranslationRotation((2.0 + 0.2, 0.0, 0.0),(0,0,0,1))
    ty = tft.fromTranslationRotation((0.0, 2.0 - 0.2, 0.0),(0,0,0,1))
    ts1 = tf.transformations.translation_from_matrix(np.dot(tf_OrignOnMap,tx))
    ts2 = tf.transformations.translation_from_matrix(np.dot(tf_OrignOnMap,ty))
    or1 = tf.transformations.quaternion_from_matrix(np.dot(tf_OrignOnMap,tx))
    or2 = tf.transformations.quaternion_from_matrix(np.dot(tf_OrignOnMap,ty))
    orignmarker1 = Marker()
    orignmarker1.header.frame_id = "map"
    orignmarker1.id = 0
    orignmarker1.ns = "orignal"
    orignmarker1.action = Marker.ADD
    orignmarker1.pose.position.x = ts1[0]
    orignmarker1.pose.position.y = ts1[1]
    orignmarker1.pose.position.z = -0.0015
    orignmarker1.pose.orientation.x = or1[0]
    orignmarker1.pose.orientation.y = or1[1]
    orignmarker1.pose.orientation.z = or1[2]
    orignmarker1.pose.orientation.w = or1[3]
    orignmarker1.type = Marker.CUBE
    orignmarker1.color.a = 1.0
    orignmarker1.color.r = 1.0
    orignmarker1.color.g = 1.0
    orignmarker1.color.b = 1.0
    orignmarker1.scale.x = 4.0 
    orignmarker1.scale.y = 0.4 
    orignmarker1.scale.z = 0.003 
    orignmarker1.lifetime = rospy.Duration()
    rospy.sleep(0.5)

    orignmarker2 = Marker()
    orignmarker2.header.frame_id = "map"
    orignmarker2.id = 1
    orignmarker2.ns = "orignal"
    orignmarker2.action = Marker.ADD
    orignmarker2.pose.position.x = ts2[0]
    orignmarker2.pose.position.y = ts2[1]
    orignmarker2.pose.position.z = -0.0015
    orignmarker2.pose.orientation.x = or2[0]
    orignmarker2.pose.orientation.y = or2[1]
    orignmarker2.pose.orientation.z = or2[2]
    orignmarker2.pose.orientation.w = or2[3]
    orignmarker2.type = Marker.CUBE
    orignmarker2.color.a = 1.0
    orignmarker2.color.r = 1.0
    orignmarker2.color.g = 1.0
    orignmarker2.color.b = 1.0
    orignmarker2.scale.x = 0.4 
    orignmarker2.scale.y = 4.0
    orignmarker2.scale.z = 0.003 
    orignmarker2.lifetime = rospy.Duration()
    return orignmarker1,orignmarker2

if __name__=='__main__':

    rospy.init_node("infrontL")
    marker_pub = rospy.Publisher("visualization_marker",Marker,queue_size=0)

    # marklist = MarkerArray()
    # blueprint = ["G","R","B","R","G","R","R"]
    blueprint = [
        ["O","O","G","R","B","R","G","R","R"],
        ["O","O","G","R","B","R","R","G","R"],
        ["O","O","B","G","R","R","G","R","R"],
        ["O","O","R","R","R","B","G","G","R"],
        ["O","O","G","R","R","R","G","R","B"],
        ["O","O","G","B","R","R","G","R","R"],
        ["O","O","G","R","R","R","B","G","R"],
        ]
    goal = []

    x = 5.0
    y = 2.0

    tolerance = 0.05
    brickindex = 0
    endZ = 0.1 - tolerance
    for j in range(0,len(blueprint)):
        endX = 0.2
        endY = 0
        for i in range(0,len(blueprint[j])):
            goal.append(BrickInfo())
            goal[brickindex].Sequence = i
        
            if blueprint[j][i] == "B":
                goal[brickindex].y = 0.0
                goal[brickindex].x = endX + 0.6 + tolerance
                endX += 1.2 + tolerance
            elif blueprint[j][i] == "G":
                goal[brickindex].y = 0.0
                goal[brickindex].x = endX + 0.3 + tolerance
                endX += 0.6 + tolerance
            elif blueprint[j][i] == "R":
                goal[brickindex].y = 0.0
                goal[brickindex].x = endX +0.15 + tolerance
                endX += 0.3 + tolerance
            elif blueprint[j][i] == "O":
                goal[brickindex].x = 0.0
                goal[brickindex].y = endY + 0.9 + tolerance 
                endY += 1.8 + tolerance 
            goal[brickindex].z =  endZ + tolerance
        
            # print(goal[i])
            brickindex += 1
        endZ += 0.2 + tolerance

    yaw = np.random.rand()
    rotat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    tf_OrignOnMap = tft.fromTranslationRotation((x,y,0),rotat)

    orignmarker1,orignmarker2 = makeLmarker(tf_OrignOnMap)
    print(marker_pub.get_num_connections() )
        # pass
    marker_pub.publish(orignmarker1)
    # rospy.sleep(0.5)
    marker_pub.publish(orignmarker2)
    # rospy.sleep(1.0)

    print(tf_OrignOnMap)
    # exit()



    thismarker = Marker()
    brickcubemarker = Marker()
    idnum = 2
    brickcnt = 0
    
    for layer in range(0,len(blueprint)) :

        for brickIndex in range(len(blueprint[layer])) :
            print (brickcnt)
            
            tf_BrickOnOrign = tft.fromTranslationRotation((goal[brickcnt].x,goal[brickcnt].y,goal[brickcnt].z),(0,0,0,1))
            brickcnt += 1
            # print goal[brickIndex]
            if goal[brickIndex].x == 0.0:
                rot_ = tf.transformations.quaternion_from_euler(0,0,0)
                tf_CarOnBrick = tft.fromTranslationRotation((-0.5,0,0),rot_)      # 砖外0.5m
            elif goal[brickIndex].y == 0.0:
                rot_ = tf.transformations.quaternion_from_euler(0,0,3.1415926/2)
                tf_CarOnBrick = tft.fromTranslationRotation((0,-0.5,0),rot_)      # 砖外0.5m
            target_tf = np.dot(np.dot(tf_OrignOnMap , tf_BrickOnOrign ), tf_CarOnBrick)
            target_rot = tf.transformations.quaternion_from_matrix(target_tf)
            target_trans = tf.transformations.translation_from_matrix(target_tf)

            brick_tf = np.dot(tf_OrignOnMap , tf_BrickOnOrign )
            brick_rot = tf.transformations.quaternion_from_matrix(brick_tf)
            brick_trans = tf.transformations.translation_from_matrix(brick_tf)

            brickcubemarker.header.seq = 0
            brickcubemarker.header.frame_id = "map"
            brickcubemarker.header.stamp    = rospy.get_rostime()
            brickcubemarker.id = idnum
            idnum += 1
            brickcubemarker.ns = "bricks"
            brickcubemarker.pose.position.x = brick_trans[0]
            brickcubemarker.pose.position.y = brick_trans[1]
            brickcubemarker.pose.position.z = brick_trans[2]
            brickcubemarker.pose.orientation.x = brick_rot[0]
            brickcubemarker.pose.orientation.y = brick_rot[1]
            brickcubemarker.pose.orientation.z = brick_rot[2]
            brickcubemarker.pose.orientation.w = brick_rot[3]
            brickcubemarker.type = Marker.CUBE
            brickcubemarker.color.a = 1.0
            brickcubemarker.color.r = 0.0
            brickcubemarker.color.g = 0.0
            brickcubemarker.color.b = 0.0
            if blueprint[layer][brickIndex] == "B":
                brickcubemarker.color.b = 1.0
                brickcubemarker.scale.x = 1.2
            elif blueprint[layer][brickIndex] == "G":
                brickcubemarker.color.g = 1.0
                brickcubemarker.scale.x = 0.6
            elif blueprint[layer][brickIndex] == "R":
                brickcubemarker.color.r = 1.0
                brickcubemarker.scale.x = 0.3
            brickcubemarker.scale.y = 0.2
            brickcubemarker.scale.z = 0.2

            if blueprint[layer][brickIndex] == "O":
                brickcubemarker.color.r = 1.0
                brickcubemarker.color.g = 0.5
                brickcubemarker.scale.y = 1.8
                brickcubemarker.scale.x = 0.2
                brickcubemarker.scale.z = 0.2
            brickcubemarker.lifetime = rospy.Duration(0)
            marker_pub.publish(brickcubemarker)
            # rospy.sleep(1.0)

        
            thismarker.header.seq = 0
            thismarker.header.frame_id = "map"

            thismarker.header.stamp    = rospy.get_rostime()
            thismarker.ns = "carpose"
            thismarker.id = idnum 
            idnum += 1
            thismarker.pose.position.x = target_trans[0]
            thismarker.pose.position.y = target_trans[1]
            thismarker.pose.position.z = 0
            thismarker.pose.orientation.x = target_rot[0]
            thismarker.pose.orientation.y = target_rot[1]
            thismarker.pose.orientation.z = target_rot[2]
            thismarker.pose.orientation.w = target_rot[3]
            thismarker.type = Marker.ARROW
            thismarker.color.a = 1.0
            thismarker.color.r = 0.0
            thismarker.color.g = 0.0
            thismarker.color.b = 0.0
            if blueprint[layer][brickIndex] == "B":
                thismarker.color.b = 1.0
            elif blueprint[layer][brickIndex] == "G":
                thismarker.color.g = 1.0
            elif blueprint[layer][brickIndex] == "R":
                thismarker.color.r = 1.0

            elif blueprint[layer][brickIndex] == "O":
                thismarker.color.r = 1.0
                thismarker.color.g = 0.5
            thismarker.scale = gmsg.Vector3(0.5,0.05,0.05)
            thismarker.lifetime = rospy.Duration(0)

            # marklist.markers.append(thismarker)

            marker_pub.publish(thismarker)
            rospy.sleep(0.5)

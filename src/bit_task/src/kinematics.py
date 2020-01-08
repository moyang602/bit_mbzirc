#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Yang Mo"
__maintainer__ = "Yang Mo"
__email__ = "moyang602@163.com"

import numpy as np
import tf.transformations as tf
from math import *
import cmath
from geometry_msgs.msg import Pose, Quaternion

"""Kinematics Docstring
   This python script provides solution of forward kinematics and 
   inverse kinematics for Universal Robot UR3/5/10.
"""

# Congifuration
# Select the robot type.
# UR10 for 'UR10'
# UR5 for 'UR5'
# UR5e for 'UR5e'
# UR3 for 'UR3'

ROBOT = 'UR5e'


# DH Parameter

if ROBOT == 'UR10':

    # d (unit: mm)
    d1 = 0.1273
    d2 = d3 = 0
    d4 = 0.163941
    d5 = 0.1157
    d6 = 0.0922

    # a (unit: mm)
    a1 = a4 = a5 = a6 = 0
    a2 = -0.612
    a3 = -0.5723

elif ROBOT == 'UR5':

    # d (unit: mm)
    d1 = 0.089159 
    d2 = d3 = 0
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823

    # a (unit: mm)
    a1 = a4 = a5 = a6 = 0
    a2 = -0.425
    a3 = -0.39225

elif ROBOT == 'UR5e':

    # SDH
    # d (unit: mm)
    d1 = 0.163 
    d2 = d3 = 0
    d4 = 0.134      #0.138-0.131+0.127
    d5 = 0.1        #0.09465    # 0.1
    d6 = 0.12737

    # a (unit: mm)
    a1 = a4 = a5 = a6 = 0
    a2 = -0.425
    a3 = -0.392     #-0.39225   # -0.392

    # # MDH
    # # d (unit: mm)
    # d1 = 0.163 
    # d2 = 0.138
    # d3 = -0.131
    # d4 = 0.127
    # d5 = 0.09465  # 0.1
    # d6 = 0.12737

    # # a (unit: mm)
    # a1 = a2 = a5 = a6 = 0
    # a3 = -0.425
    # a4 = -0.39225 # -0.392

elif ROBOT == 'UR3':

    # d (unit: mm)
    d1 = 0.1519 
    d2 = d3 = 0
    d4 = 0.11235
    d5 = 0.08535
    d6 = 0.0819

    # a (unit: mm)
    a1 = a4 = a5 = a6 = 0
    a2 = -0.24365
    a3 = -0.21325

# List type of D-H parameter
# Do not remove these
d = np.array([d1, d2, d3, d4, d5, d6]) # unit: mm
a = np.array([a1, a2, a3, a4, a5, a6]) # unit: mm
# alpha = np.array([0.0, pi/2, 0.0, 0.0, pi/2, -pi/2]) # unit: radian   MDH
alpha = np.array([pi/2, 0, 0, pi/2, -pi/2, 0]) # unit: radian   SDH
# Auxiliary Functions

def ur2ros(ur_pose):
    """Transform pose from UR format to ROS Pose format.
    Args:
        ur_pose: A pose in UR format [px, py, pz, rx, ry, rz] 
        (type: list)
    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = ur_pose[0]
    ros_pose.position.y = ur_pose[1]
    ros_pose.position.z = ur_pose[2]

    # Ros orientation
    angle = sqrt(ur_pose[3] ** 2 + ur_pose[4] ** 2 + ur_pose[5] ** 2)
    direction = [i / angle for i in ur_pose[3:6]]
    np_T = tf.rotation_matrix(angle, direction)
    np_q = tf.quaternion_from_matrix(np_T)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]
    
    return ros_pose


def ros2np(ros_pose):
    """Transform pose from ROS Pose format to np.array format.
    Args:
        ros_pose: A pose in ROS Pose format (type: Pose)
    Returns:
        An HTM (type: np.array).
    """

    # orientation
    np_pose = tf.quaternion_matrix([ros_pose.orientation.x, ros_pose.orientation.y, \
                                    ros_pose.orientation.z, ros_pose.orientation.w])
    
    # position
    np_pose[0][3] = ros_pose.position.x
    np_pose[1][3] = ros_pose.position.y
    np_pose[2][3] = ros_pose.position.z

    return np_pose


def np2ros(np_pose):
    """Transform pose from np.array format to ROS Pose format.
    Args:
        np_pose: A pose in np.array format (type: np.array)
    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = np_pose[0, 3]
    ros_pose.position.y = np_pose[1, 3]
    ros_pose.position.z = np_pose[2, 3]

    # ROS orientation 
    np_q = tf.quaternion_from_matrix(np_pose)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]

    return ros_pose

def np2rpy(np_pose):
    """Transform pose from np.array format to RPY Pose format.
    Args:
        np_pose: A pose in np.array format (type: np.array)
    Returns:
        An HTM (type: Pose).
    """
    pose = [0,0,0,0,0,0]

    pose[0] = np_pose[0, 3]
    pose[1] = np_pose[1, 3]
    pose[2] = np_pose[2, 3]

    rpy = tf.euler_from_matrix(np_pose)
    
    pose[3] = rpy[0]
    pose[4] = rpy[1]
    pose[5] = rpy[2]

    return pose


def select(q_sols, q_d, w=[1]*6):
    """Select the optimal solutions among a set of feasible joint value 
       solutions.
    Args:
        q_sols: A set of feasible joint value solutions (unit: radian)
        q_d: A list of desired joint value solution (unit: radian)
        w: A list of weight corresponding to robot joints
    Returns:
        A list of optimal joint value solution.
    """

    error = []
    for q in q_sols:
        error.append(sum([w[i] * (q[i] - q_d[i]) ** 2 for i in range(6)]))
    
    return q_sols[error.index(min(error))]


def HTM(i, theta):
    """Calculate the HTM between two links.
    Args:
        i: A target index of joint value. 
        theta: A list of joint value solution. (unit: radian)
    Returns:
        An HTM of Link l w.r.t. Link l-1, where l = i + 1.
    """

    Rot_z = np.matrix(np.identity(4))
    Rot_z[0, 0] = Rot_z[1, 1] = cos(theta[i])
    Rot_z[0, 1] = -sin(theta[i])
    Rot_z[1, 0] = sin(theta[i])

    Trans_z = np.matrix(np.identity(4))
    Trans_z[2, 3] = d[i]

    Trans_x = np.matrix(np.identity(4))
    Trans_x[0, 3] = a[i]

    Rot_x = np.matrix(np.identity(4))
    Rot_x[1, 1] = Rot_x[2, 2] = cos(alpha[i])
    Rot_x[1, 2] = -sin(alpha[i])
    Rot_x[2, 1] = sin(alpha[i])

    # A_i = Rot_x * Trans_x * Trans_z * Rot_z     # MDH
    A_i = Rot_z * Trans_z * Trans_x * Rot_x   # SDH    
    
    return A_i


# Forward Kinematics

def fwd_kin(theta, i_unit='r', o_unit='mat'):
    """Solve the HTM based on a list of joint values.
    Args:
        theta: A list of joint values. (unit: radian)
        i_unit: Output format. 'r' for radian; 'd' for degree.
        o_unit: Output format. 'mat' for np.array; 'ros' for ROS Pose; 'rpy' for RPY Pose.
    Returns:
        The HTM of end-effector joint w.r.t. base joint
    """

    T_06 = np.matrix(np.identity(4))

    if i_unit == 'd':
        theta = [radians(i) for i in theta]

    for i in range(6):
        T_06 *= HTM(i, theta)

    if o_unit == 'mat':
        return T_06
    elif o_unit == 'ros':
        return np2ros(T_06)
    elif o_unit == 'rpy':
        return np2rpy(T_06)


# Inverse Kinematics

def inv_kin(p, q_d, i_unit='r', o_unit='r'):
    """Solve the joint values based on an HTM.
    Args:
        p: A pose.
        q_d: A list of desired joint value solution 
             (unit: radian).
        i_unit: Input format. 'r' for radian; 'd' for degree.
        o_unit: Output format. 'r' for radian; 'd' for degree.
    Returns:
        A list of optimal joint value solution.
    """

    # Preprocessing
    if type(p) == Pose: # ROS Pose format
        T_06 = ros2np(p)
    elif type(p) == list: # UR format
        T_06 = ros2np(ur2ros(p))

    # print T_06
    if i_unit == 'd':
        q_d = [radians(i) for i in q_d]

    # Initialization of a set of feasible solutions
    theta = np.zeros((8, 6))

    error = [0, 0, 0, 0, 0, 0]

    nx=T_06[0,0]
    ny=T_06[1,0]
    nz=T_06[2,0]
    
    ox=T_06[0,1]
    oy=T_06[1,1]
    oz=T_06[2,1]
    
    ax=T_06[0,2]
    ay=T_06[1,2]
    az=T_06[2,2]
    
    px=T_06[0,3]
    py=T_06[1,3]
    pz=T_06[2,3]
    


    # theta1
    m = d6*ay - py 
    n = d6*ax - px
    tmp = sqrt(m**2+n**2-d4**2)
    if m**2+n**2-d4**2>=0:
        theta1 = [atan2(m,n)-atan2(d4,tmp),atan2(m,n)-atan2(d4,-tmp)]    # m^2+n^2-d4^2>0
        theta[0:4, 0] = theta1[0]
        theta[4:8, 0] = theta1[1]   
    else:
        return error


    # theta5
    theta5 = []
    for i in range(2):
        if ax*sin(theta1[i]) - ay*cos(theta1[i])<=1:
            value = acos(ax*sin(theta1[i]) - ay*cos(theta1[i])) #ax*s1-ay*c1<1
            theta5.append(value)
            theta5.append(-value)
        else:
            return error
    for i in range(4):
        theta[2*i, 4] = theta5[i]
        theta[2*i+1, 4] = theta5[i]


    # theta6
    for i in range(8):
        s1 = sin(theta[i,0])
        c1 = cos(theta[i,0])
        s5 = sin(theta[i,4])
        m = nx*s1-ny*c1
        n = ox*s1-oy*c1
        if fabs(s5)>1e-5:
            theta6 = atan2(m,n)-atan2(s5,0)         # s5 != 0
            theta[i,5] = theta6
        else:
            return error


    # theta3                        
    for i in range(4):
        s6 = sin(theta[2*i,5])
        c6 = cos(theta[2*i,5])
        c1 = cos(theta[2*i,0])
        s1 = sin(theta[2*i,0])
        m = d5*(s6*(nx*c1+ny*s1)+c6*(ox*c1+oy*s1))-d6*(ax*c1+ay*s1)+px*c1+py*s1
        n = pz-d1-az*d6+d5*(oz*c6+nz*s6)
        if m**2 +n**2 <= (a2+a3)**2:
            theta[2*i,2] = acos((m**2+n**2-a2**2-a3**2)/(2*a2*a3))                      #m^2 + n^2<(a2+a3)^2
            theta[2*i+1,2] = -acos((m**2+n**2-a2**2-a3**2)/(2*a2*a3))
        else:
            return error
        for j in range(2):
            # theta2
            s2=((a3*cos(theta[2*i+j,2])+a2)*n-a3*sin(theta[2*i+j,2])*m)/(a2**2+a3**2+2*a2*a3*cos(theta[2*i+j,2])) 
            c2=(m+a3*sin(theta[2*i+j,2])*s2)/(a3*cos(theta[2*i+j,2])+a2)
            theta[2*i+j,1] = atan2(s2,c2)

    # theta4
    for i in range(8):
        m = -sin(theta[i,5])*(nx*cos(theta[i,0])+ny*sin(theta[i,0]))-cos(theta[i,5])*(ox*cos(theta[i,0])+oy*sin(theta[i,0]))
        n = oz*cos(theta[i,5])+nz*sin(theta[i,5])
        theta[i,3]=atan2(m,n)-theta[i,1]-theta[i,2]

    for i in range(8):
        for j in range(6):
            while theta[i,j] > pi:
                theta[i,j] -= 2*pi
            while theta[i,j] <-pi:
                theta[i,j] += 2*pi

    theta = theta.tolist()

    # Select the most close solution
    q_sol = select(theta, q_d)
    # Output format
    if o_unit == 'r': # (unit: radian)
        return q_sol
    elif o_unit == 'd': # (unit: degree)
        return [degrees(i) for i in q_sol]
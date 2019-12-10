#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy

if __name__ == '__main__':

    rospy.init_node('FightFireAction', anonymous = False)

    # pub_ee = rospy.Publisher('endeffCmd',EndEffector,queue_size=1)
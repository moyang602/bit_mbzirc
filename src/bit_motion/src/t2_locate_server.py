#! /usr/bin/env python

import roslib
import rospy
import actionlib

import bit_motion.msg

class LocateServer(object):
    _feedback = bit_motion.msg.locateFeedback()
    _result = bit_motion.msg.locateResult()

    def __init__(self, name):
        self._action_name = name
        self.server = actionlib.SimpleActionServer(self._action_name, bit_motion.msg.locateAction, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()
        rospy.loginfo("%s server ready! ", name)

    def execute_cb(self, goal):
        
        # looking for bricks and building place
        self._feedback.feedback_state = "Looking for red bricks"
        self.server.publish_feedback(self._feedback)
        rospy.sleep(3.)    # wait 3s

        self._feedback.feedback_state = "Looking for green bricks"
        self.server.publish_feedback(self._feedback)
        rospy.sleep(3.)    # wait 3s

        self._feedback.feedback_state = "Looking for blue bricks"
        self.server.publish_feedback(self._feedback)
        rospy.sleep(3.)    # wait 3s

        self._feedback.feedback_state = "Looking for orange bricks"
        self.server.publish_feedback(self._feedback)
        rospy.sleep(3.)    # wait 3s

        self._feedback.feedback_state = "Looking for building place"
        self.server.publish_feedback(self._feedback)
        rospy.sleep(3.)    # wait 3s

        rospy.loginfo("Server task done")

        if self.server.is_active():
            self._result.finish_state = 1
            self.server.set_succeeded(self._result)
 
if __name__ == '__main__':
    rospy.init_node('locate_server_node')
    server = LocateServer("locateAction")
    rospy.spin()
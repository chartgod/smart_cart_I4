#!/bin/bash
import rospy
import move_base_msgs.msg
from move_base_msgs.msg import MoveBaseAction, _MoveBaseFeedback, _MoveBaseResult
import actionlib

g_feedback = _MoveBaseFeedback.MoveBaseFeedback()
g_result = _MoveBaseResult.MoveBaseResult()

def execute_cb(goal):
    r = rospy.Rate(5)
    success = True

    rospy.loginfo("position %s" % g_feedback.base_position.pose)
        #move_base_msgs.msg._MoveBaseFeedback.MoveBaseFeedback.base_position.pose

    if _as.is_preempt_requested():
        rospy.loginfo("the goal has been cancelled/preempted")
        _as.set_preempted()
        success = False
    if success:
        g_result = g_feedback
        rospy.loginfo("Succeeded")
        _as.set_succeeded(g_result)

rospy.init_node("test")
_as = actionlib.SimpleActionServer("move_base",MoveBaseAction, execute_cb, False)
_as.start()
rospy.spin()
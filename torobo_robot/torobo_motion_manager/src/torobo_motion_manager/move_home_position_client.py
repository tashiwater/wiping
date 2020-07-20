#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
import actionlib
from torobo_msgs.msg import MoveHomePositionAction, MoveHomePositionGoal, MoveHomePositionResult
import sys
from os import path 

def connect_server(nameSpace, timeout=3):
    rospy.loginfo("connect move home position action server")
    if (nameSpace[-1] != "/"):
        nameSpace += "/"
    ac = actionlib.SimpleActionClient(
        nameSpace + "move_home_position",
        MoveHomePositionAction
    )
    if (ac.wait_for_server(rospy.Duration(timeout)) == False):
        rospy.loginfo("failed to connect")
        return False
    rospy.loginfo("succeeded to connect")
    return True

def call_action(nameSpace, transitionTime, timeout=3):
    rospy.loginfo("move home position action client")
    result = MoveHomePositionResult()
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        ac = actionlib.SimpleActionClient(
            nameSpace + "move_home_position",
            MoveHomePositionAction
        )
        if (ac.wait_for_server(rospy.Duration(timeout)) == False):
            rospy.loginfo("timeout")
            return result
        goal = MoveHomePositionGoal()
        goal.transitionTime = transitionTime

        rospy.loginfo("[Move to home position service] transition time:%f" % goal.transitionTime)
        ac.send_goal(goal)
        # ac.wait_for_result()
        result = ac.get_result()
    except rospy.ServiceException, e:
        rospy.loginfo("Action call failed: %s" % e)
    return result

if __name__ == '__main__':
    if (len(sys.argv) > 2):
        nameSpace = sys.argv[1]
        transition_time = float(sys.argv[2])
        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        call_action(nameSpace, transition_time)
    else:
        print "invalid args"
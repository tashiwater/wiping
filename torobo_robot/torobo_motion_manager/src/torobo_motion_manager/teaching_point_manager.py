#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
import numpy
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from torobo_msgs.srv import DeleteTeachingPoint
from torobo_msgs.srv import GetTeachingPoint
from torobo_msgs.srv import GetTeachingPointNames
from torobo_msgs.srv import RecordTeachingPoint
from torobo_msgs.msg import MoveTeachingPointAction, MoveTeachingPointGoal, MoveTeachingPointResult

def ConvertPositionsToJointTrajectoryPoint(positions):
    point = JointTrajectoryPoint()
    for i in range(len(positions)):
        point.positions.append(positions[i])
        point.velocities.append(0.0)
        point.accelerations.append(0.0)
        point.effort.append(0.0)
    return point

def DeleteTeachingPointFromRosParam(nameSpace, tpName):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'delete_teaching_point', DeleteTeachingPoint)
        response = service(tpName)
        if response.success:
            rospy.loginfo('delete TP[%s] succeeded' % (tpName))
        else:
            rospy.loginfo('delete TP[%s] failed' % (tpName))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)

def GetTeachingPointFromRosParam(nameSpace, tpName):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'get_teaching_point', GetTeachingPoint)
        response = service(tpName)
        if response.success:
            rospy.loginfo('get TP[%s] succeeded' % (tpName))
            return response.point
        else:
            rospy.loginfo('get TP[%s] failed' % (tpName))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)
        return
    return None

def GetTeachingPointNamesFromRosParam(nameSpace):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'get_teaching_point_names', GetTeachingPointNames)
        response = service()
        if response.success:
            rospy.loginfo('get TP names succeeded')
            print response.teachingPointNames
            return response.teachingPointNames
        else:
            # Any teaching point has not created yet.
            #rospy.loginfo('get TP names failed')
            return []
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)
        return
    return None

def RecordTeachingPointToRosParam(nameSpace, tpName, point):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'record_teaching_point', RecordTeachingPoint)
        response = service(tpName, point)
        if response.success:
            rospy.loginfo('record TP[%s] succeeded' % (tpName))
        else:
            rospy.loginfo('record TP[%s] failed' % (tpName))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)

def connect_server(nameSpace, timeout=3):
    rospy.loginfo("connect move to teaching point action server")
    if (nameSpace[-1] != "/"):
        nameSpace += "/"
    ac = actionlib.SimpleActionClient(
        nameSpace + 'move_teaching_point',
        MoveTeachingPointAction
    )
    if (ac.wait_for_server(rospy.Duration(timeout)) == False):
        rospy.loginfo("failed to connect")
        return False
    rospy.loginfo("succeeded to connect")
    return True

def MoveToTeachingPoint(nameSpace, teachingPointName, transitionTime, timeout=3):
    rospy.loginfo("move to teaching point action client")
    result = MoveTeachingPointResult()
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        ac = actionlib.SimpleActionClient(
            nameSpace + 'move_teaching_point',
            MoveTeachingPointAction
        )
        if (ac.wait_for_server(rospy.Duration(timeout)) == False):
            rospy.loginfo("timeout")
            return result
        goal = MoveTeachingPointGoal()
        goal.teachingPointName = teachingPointName
        goal.transitionTime = transitionTime

        rospy.loginfo("[Move to teaching point service] teaching point name:%s, transition time:%f" % (goal.teachingPointName, goal.transitionTime))
        ac.send_goal(goal)
        # ac.wait_for_result()
        result = ac.get_result()
    except rospy.ServiceException, e:
        rospy.loginfo("Action call failed: %s" % e)
    return result

if __name__ == '__main__':
    pass
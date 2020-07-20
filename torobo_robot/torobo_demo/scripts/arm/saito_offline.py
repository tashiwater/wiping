#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
import cv2
import threading
import traceback,sys,os
from cv_bridge import CvBridge, CvBridgeError
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import Image
lock = threading.Lock()
class IMAGEBUFFER(object):
    def __init__(self):
        self._image = None

    def is_empty(self):
        # Need Lock
        with lock:
            return self._image is None

    def push_image(self, img):
        # Need Lock
        with lock:
            self._image = img

    def pop_image(self):
        # Need Lock
        with lock:
            img = self._image
            self._image = None
            return img

TIME_INTERVAL=2.5

def normalization(data, indataRange, outdataRange):
    data=(data - indataRange[0])/(indataRange[1]-indataRange[0])
    data=data*(outdataRange[1] - outdataRange[0])+outdataRange[0]
    return data

i=0
def cb_image(image):
    global i
    ImageBuffer=IMAGEBUFFER()
    bridge=CvBridge()
    try:
        ImageBuffer.push_image(bridge.imgmsg_to_cv2(image, "bgr8"))
        cv_img = ImageBuffer.pop_image()
	cv2.imwrite("/home/assimilation/cameratest/test/{:0>4d}".format(i) + ".jpg", cv_img) 
    except:
        traceback.print_exc()

def main():
    global i
    rospy.Subscriber("/image_raw", Image, cb_image)

    ACTION_SERVICE_NAME = '/torobo/arm_controller/follow_joint_trajectory'
    JOINT_NAMES = ['arm/joint_' + str(i) for i in range(1, 8)] # from joint_1 to joint_8

    try:
        # Initializes a rospy node.
        rospy.init_node('toroboarm_follow_trajectory_node')

        action_datafile=open("/home/assimilation/Desktop/offline_test/1232_5_34_140_13_0.2/Norm_Angles19.txt")
        lines=action_datafile.readlines()
        data_length=len(lines)

        for i in range(data_length-1):
            line=lines[i].split(',')
            line=[float(x) for x in line]
            #j1=normalization(float(line[0]),[-0.9,0.9],[-75,255])
            #j2=normalization(float(line[1]),[-0.9,0.9],[-120,8])
            #j3=normalization(float(line[2]),[-0.9,0.9],[-165,165])
            #j4=normalization(float(line[3]),[-0.9,0.9],[-30,130])
            #j5=normalization(float(line[4]),[-0.9,0.9],[-165,165])
            #j6=normalization(float(line[5]),[-0.9,0.9],[-80,80])
            #j7=normalization(float(line[6]),[-0.9,0.9],[-80,5])
            # Executes an action.
            result = follow_trajectory(
                action_service_name = ACTION_SERVICE_NAME,
                joint_names = JOINT_NAMES,
                positions = np.radians(line),
                time_from_start = TIME_INTERVAL
            )
            rospy.loginfo(result)

    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("finished.")


##
# @brief  Function for publishing action to move the arm
# @param action_service_name  ActionService' name
# @param joint_names  list of joint names
# @param positions  list of joint's goal positions(radian)
# @param time_from_start  goal time from start
# @return  ActionService's result
# @note  
def follow_trajectory(action_service_name, joint_names, positions, time_from_start):

    # Creates the SimpleActionClient.
    ac = actionlib.SimpleActionClient(action_service_name, FollowJointTrajectoryAction)

    # Waits until the action server has started up.
    ac.wait_for_server()

    # Creates a goal.
    goal = FollowJointTrajectoryGoal()
    goal.goal_time_tolerance = rospy.Time(1.0)
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0 for i in range(len(joint_names))]
    point.accelerations = [0.0 for i in range(len(joint_names))]
    point.effort = [0.0 for i in range(len(joint_names))]
    point.time_from_start = rospy.Duration(time_from_start)
    goal.trajectory.points.append(point)

    # Sends the goal.
    ac.send_goal(goal)

    # Waits for the server.
    finished_before_timeout = ac.wait_for_result(timeout=rospy.Duration(time_from_start + 2.0))

    # Returns result.
    if finished_before_timeout:
        return ac.get_result()
    else:
        return None

if __name__ == '__main__':
    main()


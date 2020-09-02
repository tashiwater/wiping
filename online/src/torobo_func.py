#!/usr/bin/env python
# coding: utf-8

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

##
# @brief  Function for publishing action to move the arm
# @param action_service_name  ActionService' name
# @param joint_names  list of joint names
# @param positions  list of joint's goal positions(radian)
# @param time_from_start  goal time from start
# @return  ActionService's result
# @note
def follow_trajectory(action_service_name, joint_names, positions, time_from_start=1):

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
    finished_before_timeout = ac.wait_for_result(
        timeout=rospy.Duration(time_from_start + 2.0)
    )

    # Returns result.
    if finished_before_timeout:
        return ac.get_result()
    else:
        return None

#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


def main():

    ACTION_SERVICE_NAME = '/torobo/arm_controller/follow_joint_trajectory'
    JOINT_NAMES = ['arm/joint_' + str(i) for i in range(1, 8)] # from joint_1 to joint_8

    try:
        # Initializes a rospy node.
        rospy.init_node('toroboarm_follow_trajectory_node')

        # Executes an action.
        result = follow_trajectory(
            action_service_name = ACTION_SERVICE_NAME,
            joint_names = JOINT_NAMES,
            positions = np.radians([0.0, 0.0, 0.0, 0.0, 10.0, 10.0, 0.0]),
            time_from_start = 1
        )
        rospy.loginfo(result)

        # Executes an action.
        result = follow_trajectory(
            action_service_name = ACTION_SERVICE_NAME,
            joint_names = JOINT_NAMES,
            positions = np.radians([0.0, 0.0, 0.0, 0.0, 30.0, 30.0, 0.0]),
            time_from_start = 1
        )
        rospy.loginfo(result)

	# Executes an action.
        result = follow_trajectory(
            action_service_name = ACTION_SERVICE_NAME,
            joint_names = JOINT_NAMES,
            positions = np.radians([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            time_from_start = 0.1
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


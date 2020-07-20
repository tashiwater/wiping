#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import actionlib
import numpy as np
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


def main():

    ACTION_SERVICE_NAME = '/torobo/gripper_controller/gripper_cmd'

    try:
        # Initializes a rospy node.
        rospy.init_node('toroboarm_gripper_command_node')

        # open gripper.
        # result = gripper_command(ACTION_SERVICE_NAME, np.radians(70.0), -10.0) # revolute joint
        result = gripper_command(ACTION_SERVICE_NAME, 0.07, -10.0) # prismatic joint
        rospy.sleep(2.0)
        rospy.loginfo(result)

        # close gripper.
        #result = gripper_command(ACTION_SERVICE_NAME, np.radians(10.0), 10.0) # revolute joint
        result = gripper_command(ACTION_SERVICE_NAME, 0.01, 10.0) # prismatic joint
        rospy.sleep(2.0)
        rospy.loginfo(result)

    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("finished.")


##
# @brief  Function for publishing action to move the gripper
# @param action_service_name  ActionService' name
# @param position  position of finger. 
# @param max_effort  max effort for grasp
# @return  ActionService's result
# @note  Note that position's unit is "meter" and max_effort's unit is "N".
def gripper_command(action_service_name, position, max_effort):

    # Creates the SimpleActionClient.
    ac = actionlib.SimpleActionClient(action_service_name, GripperCommandAction)

    # Waits until the action server has started up.
    ac.wait_for_server()

    # Creates a goal.
    goal = GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = max_effort

    # Sends the goal.
    ac.send_goal(goal)

    # Waits for the server.
    finished_before_timeout = ac.wait_for_result(timeout=rospy.Duration(3.0))
    
    # Returns result.
    if finished_before_timeout:
        return ac.get_result()
    else:
        return None


if __name__ == '__main__':
    main()


#!/usr/bin/env python


import roslaunch
import rospy
import sys
import yaml
import argparse
import os


def main(args):

    def launch_teaching_point_node(controller_name):
        node = roslaunch.core.Node(
            package=args.pkg,
            node_type=args.type_tp,
            name='torobo_' + controller_name + '_teaching_point_node',
            namespace=args.ns + '/' + controller_name + '_controller',
            respawn=False,
            output=args.output,
        )
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
        print process.is_alive()
        # process.stop()

    def launch_teaching_trajectory_node(controller_name):
        node = roslaunch.core.Node(
            package=args.pkg,
            node_type=args.type_traj,
            name='torobo_' + controller_name + '_teaching_trajectory_node',
            namespace=args.ns + '/' + controller_name + '_controller',
            respawn=False,
            output=args.output,
        )
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
        print process.is_alive()
        # process.stop()

    rospy.init_node('torobo_motion_manager_spawner_node', anonymous=True)

    robot_config_model = rospy.get_param('/torobo/robot_config/model')

    if robot_config_model == 'arm':
        launch_teaching_point_node('arm')
        launch_teaching_trajectory_node('arm')
    elif robot_config_model == 'arm_gripper':
        launch_teaching_point_node('arm')
        launch_teaching_trajectory_node('arm')
    elif robot_config_model == 'humanoid':
        launch_teaching_point_node('left_arm')
        launch_teaching_trajectory_node('left_arm')
        launch_teaching_point_node('right_arm')
        launch_teaching_trajectory_node('right_arm')
        launch_teaching_point_node('torso_head')
        launch_teaching_trajectory_node('torso_head')
    elif robot_config_model == 'humanoid_gripper':
        launch_teaching_point_node('left_arm')
        launch_teaching_trajectory_node('left_arm')
        launch_teaching_point_node('right_arm')
        launch_teaching_trajectory_node('right_arm')
        launch_teaching_point_node('torso_head')
        launch_teaching_trajectory_node('torso_head')

    rospy.spin()


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--pkg", type=str, default="")
    parser.add_argument("--type_tp", type=str, default="")
    parser.add_argument("--type_traj", type=str, default="")
    parser.add_argument("--ns", type=str, default="")
    parser.add_argument("--name", type=str, default="")
    parser.add_argument("--output", type=str, default="")
    #parser.add_argument("--robot_config", type=str, default="")
    return parser.parse_args(args)


if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)

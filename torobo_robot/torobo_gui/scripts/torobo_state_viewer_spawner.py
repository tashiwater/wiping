#!/usr/bin/env python


import roslaunch
import rospy
import sys
import yaml
import argparse
import os


def main(args):
    rospy.init_node('torobo_state_viewer_spawner_node', anonymous=True)

    robot_config_model = rospy.get_param('robot_config/model')

    if robot_config_model == 'arm':
        perspective = os.path.join(args.perspective_dir, 'toroboarm_state_viewer.perspective')
    elif robot_config_model == 'arm_gripper':
        perspective = os.path.join(args.perspective_dir, 'toroboarm_state_viewer.perspective')
    elif robot_config_model == 'humanoid':
        perspective = os.path.join(args.perspective_dir, 'torobo_state_viewer.perspective')
    elif robot_config_model == 'humanoid_gripper':
        perspective = os.path.join(args.perspective_dir, 'torobo_state_viewer.perspective')

    node = roslaunch.core.Node(
        package=args.pkg,
        node_type=args.type,
        name=args.name,
        namespace=rospy.get_namespace(),
        respawn=False,
        output=args.output,
        args='--perspective-file ' + perspective
    )

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)
    print process.is_alive()
    # process.stop()

    rospy.spin()


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--pkg", type=str, default="")
    parser.add_argument("--type", type=str, default="")
    parser.add_argument("--name", type=str, default="")
    parser.add_argument("--output", type=str, default="")
    parser.add_argument("--perspective_dir", type=str, default="")
    return parser.parse_args(args)


if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)

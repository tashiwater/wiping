#!/usr/bin/env python


import roslaunch
import rospy
import sys
import yaml
import argparse
import os
from nodelet.srv import NodeletLoad


def main(args):

    def load_nodelet(name, controller):
        try:
            service_name = args.manager + '/load_nodelet'
            rospy.wait_for_service(service_name)
            service = rospy.ServiceProxy(service_name, NodeletLoad)
            res = service(name, args.type, [], [], ['--config=' + controller,], '')
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed :%s", str(e))

    rospy.init_node('torobo_driver_spawner_node', anonymous=True)

    robot_config_model = rospy.get_param('robot_config/model')

    if robot_config_model == 'arm':
        subdir = 'toroboarm'
    elif robot_config_model == 'arm_gripper':
        subdir = 'toroboarm_gripper'
    if robot_config_model == 'humanoid':
        subdir = 'torobo'
    elif robot_config_model == 'humanoid_gripper':
        subdir = 'torobo_gripper'
    

    if args.mock == "true":
        driver_config_file = "torobo_controller_mock_config.yaml"
    else:
        driver_config_file = "torobo_controller_config.yaml"

    f = open(os.path.join(args.driver_config_dir, subdir, driver_config_file))
    driver_config = yaml.load(f)
    f.close()
    from rosparam import upload_params
    # rospy.set_param('', driver_config)
    upload_params('', driver_config)

    if robot_config_model == 'arm':
        load_nodelet('toroboarm_driver', 'arm_controller')
    elif robot_config_model == 'arm_gripper':
        load_nodelet('toroboarm_driver', 'arm_gripper_controller')
    elif robot_config_model == 'humanoid':
        load_nodelet('torobo_left_arm_driver', 'left_arm_controller')
        load_nodelet('torobo_right_arm_driver', 'right_arm_controller')
        load_nodelet('torobo_torso_head_driver', 'torso_head_controller')
    elif robot_config_model == 'humanoid_gripper':
        load_nodelet('torobo_left_arm_driver', 'left_arm_gripper_controller')
        load_nodelet('torobo_right_arm_driver', 'right_arm_gripper_controller')
        load_nodelet('torobo_torso_head_driver', 'torso_head_controller')


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--manager", type=str, default="")
    parser.add_argument("--type", type=str, default="")
    parser.add_argument("--mock", type=str, default="")
    parser.add_argument("--driver_config_dir", type=str, default="")
    return parser.parse_args(args)


if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)

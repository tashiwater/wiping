#!/usr/bin/env python


import roslaunch
import rospy
import sys
import yaml
import argparse
import os
from nodelet.srv import NodeletLoad


def main(args):

    def load_nodelet(nodelet_name, nodelet_arg_array):
        try:
            service_name = args.manager + '/load_nodelet'
            rospy.wait_for_service(service_name)
            service = rospy.ServiceProxy(service_name, NodeletLoad)
            res = service(nodelet_name, args.type, [], [], nodelet_arg_array, '')
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed :%s", str(e))

    rospy.init_node('torobo_dynamics_spawner_node', anonymous=True)

    robot_config_model = rospy.get_param('robot_config/model')

    if robot_config_model == 'arm':
        if(args.sim == "true"):
            load_nodelet('torobo_dynamics', ['--sim=' + args.sim, '--model=toroboarm'])
        else:
            pass
    elif robot_config_model == 'arm_gripper':
        if(args.sim == "true"):
            load_nodelet('torobo_dynamics', ['--sim=' + args.sim, '--model=toroboarm'])
        else:
            pass
    elif robot_config_model == 'humanoid':
        load_nodelet('torobo_dynamics', ['--sim=' + args.sim, '--model=torobo'])
    elif robot_config_model == 'humanoid_gripper':
        load_nodelet('torobo_dynamics', ['--sim=' + args.sim, '--model=torobo'])


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--manager", type=str, default="")
    parser.add_argument("--type", type=str, default="")
    parser.add_argument("--sim", type=str, default="true")
    return parser.parse_args(args)

if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)

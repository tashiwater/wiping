#!/usr/bin/env python

import rospy
import sys
import yaml
import argparse
import os

import xml.dom.minidom
from math import pi


# def load_home_position(key='/torobo/robot_description_semantic', use_smallest_joint_limits=True):
def load_home_position(description):
    # use_small = use_smallest_joint_limits
    # use_mimic = True

    # Code inspired on the joint_state_publisher package by David Lu!!!
    # https://github.com/ros/robot_model/blob/indigo-devel/
    # joint_state_publisher/joint_state_publisher/joint_state_publisher
    #description = rospy.get_param(key)
    robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
    # free_joints = {}
    # dependent_joints = {}

    # Find all non-fixed joints
    output = {}
    for child in robot.childNodes:
        if child.nodeType is child.TEXT_NODE:
            continue
        if child.localName != 'group_state':
            continue

        name = child.getAttribute('name')

        if name != 'home_position':
            continue

        group = child.getAttribute('group')

        joints = child.getElementsByTagName('joint')
        home_position = {}
        for joint in joints:
            joint_name = joint.getAttribute('name')
            joint_value = joint.getAttribute('value')
            home_position[joint_name] = float(joint_value)

        output[group + '_controller/home_position'] = home_position

    f = open(args.dump_file, "w")
    f.write(yaml.dump(output, default_flow_style=False))
    f.close()


def main(args):
    if os.path.exists(args.dump_file):
        os.remove(args.dump_file)

    f = open(os.path.join(args.robot_config) , 'r')
    robot_config = yaml.load(f)
    f.close()

    f = open(os.path.join(args.model_dir, robot_config['model'], args.srdf))
    srdf_text = f.read()
    f.close()

    load_home_position(srdf_text)

    print srdf_text


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_config", type=str, default="")
    parser.add_argument("--srdf", type=str, default="")
    parser.add_argument("--model_dir", type=str, default="")
    parser.add_argument("--dump_file", type=str, default="/tmp/tmp.yaml")
    return parser.parse_args(args)


if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)

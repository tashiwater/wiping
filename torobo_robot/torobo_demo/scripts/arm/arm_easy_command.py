#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from torobo_msgs.srv import BrakeOff
import sys
from os import path
from torobo_driver import torobo_easy_command

if __name__ == '__main__':
    rospy.init_node('torobo_easy_command_node')

    torobo_easy_command.SendEasyCommandText("/torobo/arm_controller", "s 4 1")


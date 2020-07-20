#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from torobo_msgs.srv import *


def main():
    rospy.init_node('torobo_msgs_node')

    control_mode_name = 'external_force_following'
    joint_names = ['arm/joint_4', 'arm/joint_6']

    # Set control mode
    service_name = '/torobo/arm_controller/set_control_mode'
    rospy.wait_for_service(service_name)
    service = rospy.ServiceProxy(service_name, SetControlMode)
    try:
        response = service(control_mode_name, joint_names)
        rospy.loginfo('servo on [%s] result is %s' % (joint_names, response.success))
    except rospy.ServiceException, e:
        rospy.loginfo('Servoce call failes: %s' % e)

    # Set servo on
    service_name = '/torobo/arm_controller/servo_on'
    rospy.wait_for_service(service_name)
    service = rospy.ServiceProxy(service_name, ServoOn)
    try:
        response = service(joint_names)
        rospy.loginfo('servo on [%s] result is %s' % (joint_names, response.success))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)


if __name__ == '__main__':
    main()


#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
import moveit_commander

def main():
    rospy.init_node("print_cur_pose_node")

    robot = moveit_commander.RobotCommander()
    larm = moveit_commander.MoveGroupCommander("left_arm")

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        print "=" * 10
        print larm.get_current_pose().pose    
        r.sleep()
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
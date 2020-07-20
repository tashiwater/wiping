#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
import numpy
import moveit_commander
import geometry_msgs.msg
# import actionlib

def main():
    rospy.init_node("moveit_command_sender")

    robot = moveit_commander.RobotCommander()
    
    print "=" * 10, " Robot Groups:"
    print robot.get_group_names()

    # print "=" * 10, " Printing robot state"
    # print robot.get_current_state()
    # print "=" * 10 

    larm = moveit_commander.MoveGroupCommander("left_arm")

    # print "=" * 15, " Left ight arm ", "=" * 15
    # print "=" * 10, " Reference frame: %s" % larm.get_planning_frame()
    # print "=" * 10, " Reference frame: %s" % larm.get_end_effector_link()

    #Left Arm Initial Pose
    larm_initial_pose = larm.get_current_pose().pose    
    print "=" * 10, " Printing Left Hand initial pose "
    print larm_initial_pose

    # target_pose = geometry_msgs.msg.Pose()
    target_pose = larm_initial_pose
    # target_pose.position.x = larm_initial_pose.position.x - 0.1
    target_pose.position.y -= 0.01
    # target_pose.position.z = larm_initial_pose.position.z - 0.1
    # target_pose.position.y = -0.182271241593-0.3
    # target_pose.position.z = 0.0676272396419+0.3
    # target_pose.orientation.x = -0.000556712307053
    # target_pose.orientation.y = -0.706576742941
    # target_pose.orientation.z = -0.00102461782513
    # target_pose.orientation.w = 0.707635461636
    larm.set_pose_target(target_pose)

    print "=" * 10," plan1 ..."
    larm.go()
    rospy.sleep(1)

    #Clear pose
    # larm.clear_pose_targets()

    #Right Hand
    # target_pose_r.position.x = 0.221486843301
    # target_pose_r.position.y = -0.0746407547512
    # target_pose_r.position.z = 0.642545484602
    # target_pose_r.orientation.x = 0.0669013615474
    # target_pose_r.orientation.y = -0.993519060661
    # target_pose_r.orientation.z = 0.00834224628291
    # target_pose_r.orientation.w = 0.0915122442864
    # rarm.set_pose_target(target_pose_r)

    # print "=" * 10, " plan3..."
    # rarm.go()
    # rospy.sleep(1)

    # print "=" * 10,"Initial pose ..."
    # rarm.set_pose_target(rarm_initial_pose)
    # larm.set_pose_target(larm_initial_pose)
    # rarm.go()
    # larm.go()
    # rospy.sleep(2)
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
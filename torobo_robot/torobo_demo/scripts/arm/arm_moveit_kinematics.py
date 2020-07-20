#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
import moveit_commander
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest


def main():

    torobo = Torobo(movegroup="arm", tooltip="arm/link_7")
    res = torobo.compute_fk(joint_angles=np.radians([50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 0.0]))
    res = torobo.compute_ik(x=res[0], y=res[1], z=res[2], roll=res[3], pitch=res[4], yaw=res[5])


class Torobo:

    def __init__(self, movegroup, tooltip):
        """constructor of this class
            @movegroup : Specify move group's name(string) you want to operate. Selectable move group is written in moveit_config's srdf file.
            @tooltip   : Specify tooltip's name(string) you want to set position.
        """

        rospy.init_node("toroboarm_moveit_kinematics_node", anonymous=True)
        rospy.Rate(1)
        rospy.sleep(1)

        # load MoveGroupCommander by string('arm') registerd in 'toroboarm_gripper.srdf'
        self._robot = moveit_commander.MoveGroupCommander(movegroup)
        self._active_joints = self._robot.get_active_joints()
        self._tooltip = tooltip

        # load fk/ik service
        rospy.loginfo("loading fk")
        rospy.wait_for_service('compute_fk')
        rospy.wait_for_service('compute_ik')
        try:
            self._moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
            self._moveit_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)


    def compute_fk(self, joint_angles):
        """This function computes forward kinematics
            @joint_angles : joint's angles [rad]
        """

        rospy.loginfo("---- compute fk ----")

        assert len(joint_angles) == len(self._active_joints), "joint_angles doesn't matches active_joints"

        # print input
        rospy.loginfo("[input]")
        for i in range(len(self._active_joints)):
            rospy.loginfo("    {0} : {1:.2f}".format(self._active_joints[i], np.degrees(joint_angles[i])))

        # create messages
        header = Header(0, rospy.Time.now(), "/world")
        rs = RobotState()
        rs.joint_state.name = self._active_joints
        rs.joint_state.position = joint_angles

        # get forward kinematics solution
        fk_result = self._moveit_fk(header, [self._tooltip], rs)
        assert len(fk_result.pose_stamped) > 0, "fk has no solution"
        pose = fk_result.pose_stamped[0].pose
        euler = tf.transformations.euler_from_quaternion(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        )

        # print result
        rospy.loginfo("[output]")
        rospy.loginfo("    x     : {:.2f}".format(pose.position.x))
        rospy.loginfo("    y     : {:.2f}".format(pose.position.y))
        rospy.loginfo("    z     : {:.2f}".format(pose.position.z))
        rospy.loginfo("    roll  : {:.2f}".format(np.degrees(euler[0])))
        rospy.loginfo("    pitch : {:.2f}".format(np.degrees(euler[1])))
        rospy.loginfo("    yaw   : {:.2f}".format(np.degrees(euler[2])))

        return pose.position.x, pose.position.y, pose.position.z, euler[0], euler[1], euler[2]


    def compute_ik(self, x, y, z, roll, pitch, yaw):
        """This function computes inverse kinematics
            @x     : tooltip's x [m]
            @y     : tooltip's y [m]
            @z     : tooltip's z [m]
            @roll  : tooltip's roll [rad]
            @pitch : tooltip's pitch [rad]
            @yaw   : tooltip's yaw [rad]
        """

        rospy.loginfo("---- compute ik ----")

        # print input
        rospy.loginfo("[input]")
        rospy.loginfo("    x     : {:.2f}".format(x))
        rospy.loginfo("    y     : {:.2f}".format(y))
        rospy.loginfo("    z     : {:.2f}".format(z))
        rospy.loginfo("    roll  : {:.2f}".format(np.degrees(roll)))
        rospy.loginfo("    pitch : {:.2f}".format(np.degrees(pitch)))
        rospy.loginfo("    yaw   : {:.2f}".format(np.degrees(yaw)))

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        # create request
        req = PositionIKRequest()
        req.timeout = rospy.Duration(5.0)
        req.ik_link_name = self._tooltip
        req.pose_stamped.header.frame_id = "/world"
        req.pose_stamped.pose.position.x = x
        req.pose_stamped.pose.position.y = y
        req.pose_stamped.pose.position.z = z
        req.pose_stamped.pose.orientation.x = quaternion[0]
        req.pose_stamped.pose.orientation.y = quaternion[1]
        req.pose_stamped.pose.orientation.z = quaternion[2]
        req.pose_stamped.pose.orientation.w = quaternion[3]
        req.robot_state.joint_state.name = self._active_joints
        req.robot_state.joint_state.position = self._robot.get_current_joint_values() # current values as seeds
        req.group_name = self._robot.get_name()
        req.avoid_collisions = False

        # get ik result
        ik_result = self._moveit_ik(req).solution

        # print result
        rospy.loginfo("[output]")
        for i in range(len(ik_result.joint_state.name)):
            joint_name = ik_result.joint_state.name[i]
            if joint_name in self._active_joints:
                rospy.loginfo("    {0} : {1:.2f}".format(joint_name, np.degrees(ik_result.joint_state.position[i])))

        return ik_result.joint_state


if __name__ == '__main__':
    main()


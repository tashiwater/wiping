/**
 * @file  ToroboJointTrajectoryController.h
 * @brief ToroboJointTrajectoryController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_JOINT_TRAJECTORY_CONTROLLER_H__
#define __TOROBO_JOINT_TRAJECTORY_CONTROLLER_H__


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include "ToroboAbstractActionController.h"


class ToroboJointTrajectoryController : public ToroboAbstractActionController
{
public:
    ToroboJointTrajectoryController(ros::NodeHandle& node, std::string name, std::string action_name);
    virtual ~ToroboJointTrajectoryController();

protected:
    void actionCallback();
    void callback(const sensor_msgs::JointState::ConstPtr& msg);

    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    ros::ServiceClient service_cancel_trajectory_client_;

    control_msgs::FollowJointTrajectoryResult result_;
    ros::Time goal_time_ = ros::Time();
    ros::Duration goal_time_tolerance_ = ros::Duration(0.0);
    std::map<std::string, double> jointgoal_map_;
    std::map<std::string, double> tolerance_map_;

    static ros::Duration GOAL_TIME_TOLERANCE_MARGIN;
    static double GOAL_TOLERANCE_MARGIN;
};


#endif


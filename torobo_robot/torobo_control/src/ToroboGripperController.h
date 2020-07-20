/**
 * @file  ToroboGripperController.h
 * @brief ToroboGripperController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_GRIPPER_CONTROLLER_H__
#define __TOROBO_GRIPPER_CONTROLLER_H__


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include "ToroboAbstractActionController.h"


class ToroboGripperController : public ToroboAbstractActionController
{
public:
    ToroboGripperController(ros::NodeHandle& node, std::string name, std::string action_name);
    virtual ~ToroboGripperController();

protected:
    void actionCallback();
    void callback(const sensor_msgs::JointState::ConstPtr& msg);

    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;

    control_msgs::GripperCommandResult result_;
    ros::Time goal_time_ = ros::Time();
    int velocity_zero_counter_ = 0;
    double goal_position_ = 0.0;
    double max_effort_ = 0.0;

protected:
    static std::string FINGER_JOINT_NAME;
    static ros::Duration GOAL_TIME_FROM_START;
    static double GOAL_TOLERANCE;
    static double EFFORT_RATIO;
    static double VELOCITY_THRESHOLD;
    static int VELOCITY_ZERO_COUNTER_THRESHOLD;
};


#endif


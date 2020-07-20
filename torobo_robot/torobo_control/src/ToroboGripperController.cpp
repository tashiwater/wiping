/**
 * @file  ToroboGripperController.h
 * @brief ToroboGripperController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <boost/bind.hpp>
#include "ToroboGripperController.h"
#include "torobo_msgs/GetJointState.h"

using namespace std;

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/
ros::Duration ToroboGripperController::GOAL_TIME_FROM_START = ros::Duration(3.0);
double ToroboGripperController::GOAL_TOLERANCE = 0.0005; //[m]
double ToroboGripperController::VELOCITY_THRESHOLD = 0.0001; //[m/s]
std::string ToroboGripperController::FINGER_JOINT_NAME = "finger_joint";
int ToroboGripperController::VELOCITY_ZERO_COUNTER_THRESHOLD = 10;

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboGripperController::ToroboGripperController(ros::NodeHandle& node, std::string name, std::string action_name) :
    ToroboAbstractActionController(node, name, action_name),
    as_(nh_, name + "/" + action_name, false)
{
    // Set Publisher and Subscriber
    pub_ = nh_.advertise<control_msgs::GripperCommand>(nh_.getNamespace() + "/" + name_ + "/command", 1, this);
    sub_ = nh_.subscribe(nh_.getNamespace() + "/joint_state_server/joint_states", 1, &ToroboGripperController::callback, this);

    // Register ActionServer's Callbacks
    as_.registerGoalCallback(boost::bind(&ToroboGripperController::actionCallback, this));
    as_.start();
}

ToroboGripperController::~ToroboGripperController()
{
    as_.shutdown();
    pub_.shutdown();
    sub_.shutdown();
}

/*----------------------------------------------------------------------
 Protected Method Implementations
 ----------------------------------------------------------------------*/
void ToroboGripperController::callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Check if the action is active
    if(!as_.isActive()) /* 'isActive()' returns True if action server has received a goal and action server's status is "Active" or "Preempting". */
    {
        return;
    }

    if (as_.isPreemptRequested() || !ros::ok())
    {
        /****************************************************
             "isPreemptRequested" is set true
            both when a user requests preempt
            and when a user send multiple goals.
        ****************************************************/
        // [NOTE] There is no cancel command for Gripper Command.
        //        because it takes very short time to move gripper.
        ROS_WARN("%s: Preempted", action_name_.c_str());
        as_.setPreempted(); /* action server's status is changed into "Preempted"(cancel is completed). */
        return;
    }

    // Check if the goal_time is passed.
    if(ros::Time::now() > goal_time_)
    {
        // return fail succeeded signal
        result_.stalled = false;
        result_.reached_goal = false;
        as_.setAborted(result_);  // set status to "succeeded"
        ROS_WARN("%s: Aborted. : goal time tolerance is violated.", action_name_.c_str());
        return;
    }

    // Get index of joint_name in JointState
    std::string joint_name = name_.substr(0, name_.size()-11) + "/" + FINGER_JOINT_NAME;
    size_t index = std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), joint_name));

    // Check if joint_name is found in JointState.
    if (index == msg->name.size())
    {
        velocity_zero_counter_ = 0;
        return; // the condition is not satisfied.
    }

    double velocity = msg->velocity[index];
    result_.position = msg->position[index];
    result_.effort = msg->effort[index];

    // Check finger movement is stopped.
    bool flag = true;
    if(fabs(velocity) < VELOCITY_THRESHOLD)
    {
        velocity_zero_counter_++;
        if(velocity_zero_counter_ < VELOCITY_ZERO_COUNTER_THRESHOLD)
        {
            flag = false;
        }
    }
    else
    {
        velocity_zero_counter_ = 0;
        flag = false;
    }

    if (flag)
    {
        // Check "reached_goal" and "stalled" by evaluating position and effort.
        if(goal_position_ - GOAL_TOLERANCE < result_.position && result_.position < goal_position_ + GOAL_TOLERANCE)
        {
            result_.stalled = false;
            result_.reached_goal = true;
        }
        else
        {
            result_.stalled = true;
            result_.reached_goal = false;
        }
        // return result
        as_.setSucceeded(result_);  /* action server's status is changed into "Succeeded". */
        ROS_INFO("%s: Succeeded. successfuly completed [position:%f, effort:%f, stalled:%d, reached_goal:%d]", action_name_.c_str(), result_.position, result_.effort, result_.stalled, result_.reached_goal);
        return;
    }
}

void ToroboGripperController::actionCallback()
{
    /*****************************************************************
      This callback is called by action server
      when a user send a goal.
      If the user send a new goal when an old goal is active yet,
      PreemptCB is called and then GoalCB is called by action server.
      This ROS's specification is written in http://wiki.ros.org/joint_trajectory_controller
     *****************************************************************/

    ROS_INFO("Goal Received");

    if(as_.isActive()) /* 'isActive()' returns True if action server has received a goal and action server's status is "Active" or "Preempting". */
    {
        /* this block is never reached because PreemptCB is called before GoalCB when double goal is thrown. */
        ROS_WARN("New goal has been ignored because old goal is active.");
        return;
    }

    // Start Action
    control_msgs::GripperCommand command = as_.acceptNewGoal()->command;  /* action server's status is changed into "Active". */
    pub_.publish(command);

    // Set Monitoring Value
    goal_position_ = command.position;
    max_effort_ = command.max_effort;
    goal_time_ = ros::Time::now() + GOAL_TIME_FROM_START;
    velocity_zero_counter_ = 0;

}

/**
 * @file  ToroboJointTrajectoryController.cpp
 * @brief ToroboJointTrajectoryController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <boost/bind.hpp>
#include "ToroboJointTrajectoryController.h"
#include "torobo_msgs/CancelTrajectory.h"
#include "torobo_msgs/GetJointState.h"


using namespace std;

/*----------------------------------------------------------------------
 Protected Global Variables
 ----------------------------------------------------------------------*/
ros::Duration ToroboJointTrajectoryController::GOAL_TIME_TOLERANCE_MARGIN = ros::Duration(0.5);
double ToroboJointTrajectoryController::GOAL_TOLERANCE_MARGIN = 1.0 * M_PI / 180.0;

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboJointTrajectoryController::ToroboJointTrajectoryController(ros::NodeHandle& node, std::string name, std::string action_name) :
    ToroboAbstractActionController(node, name, action_name),
    as_(nh_, name + "/" + action_name, false)
{
    // Set Publisher and Subscriber and Service
    pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(nh_.getNamespace() + "/" + name_ + "/command", 1, this);
    sub_ = nh_.subscribe(nh_.getNamespace() + "/joint_state_server/joint_states", 1, &ToroboJointTrajectoryController::callback, this);
    service_cancel_trajectory_client_ = nh_.serviceClient<torobo_msgs::CancelTrajectory>(nh_.getNamespace() + "/" + name_ + "/cancel_trajectory");

    // Register ActionServer's Callbacks
    as_.registerGoalCallback(boost::bind(&ToroboJointTrajectoryController::actionCallback, this));
    as_.start(); /* action server's status is "Pending" first. */
}

ToroboJointTrajectoryController::~ToroboJointTrajectoryController()
{
    as_.shutdown();
    pub_.shutdown();
    sub_.shutdown();
    service_cancel_trajectory_client_.shutdown();
}

/*----------------------------------------------------------------------
 Protected Method Implementations
 ----------------------------------------------------------------------*/
void ToroboJointTrajectoryController::callback(const sensor_msgs::JointState::ConstPtr& msg)
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

        // send cancel_trajectory to ToroboDriver
        torobo_msgs::CancelTrajectory srv;
        for (auto itr = jointgoal_map_.begin(); itr != jointgoal_map_.end(); ++itr)
        {
            std::string joint_name = itr->first;
            srv.request.joint_names.push_back(joint_name);
        }
        if(service_cancel_trajectory_client_.call(srv))
        {
            ROS_INFO("%s: cancel service is called", action_name_.c_str());
        }
        ROS_WARN("%s: Preempted", action_name_.c_str());
        as_.setPreempted(); /* action server's status is changed into "Preempted"(cancel is completed). */

        return;
    }

    // Check if the goal_time with tolerance is passed over
    if(ros::Time::now() > goal_time_ + goal_time_tolerance_)
    {
        // return fail succeeded signal
        result_.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
        result_.error_string = action_name_ + ": Failed.  goal tolerance is violated.";
        as_.setSucceeded(result_);  // set status to "succeeded"
        return;
    }

    // Check if the current joint_data is equal to goal by allowing tolerance value.
    bool flag = true;
    for (auto itr = jointgoal_map_.begin(); itr != jointgoal_map_.end(); ++itr)
    {
        std::string joint_name = itr->first;
        double goal = itr->second;
        double tolerance = tolerance_map_[joint_name];

        // Get index of joint_name in JointState
        size_t index = std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), joint_name));

        // Check if joint_name is found in JointState.
        if (index == msg->name.size())
        {
            flag = false;
            break; // the condition is not satisfied.
        }

        // Check if current joint position is equal to goal.
        double current_position = msg->position[index];
        if(!(goal - tolerance < current_position && current_position < goal + tolerance))
        {
            flag = false;
            break; // the condition is not satisfied.
        }
    }

    if(flag)
    {
        // If all the condition is passed, return success signal, and finish monitoring.
        result_.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        result_.error_string = action_name_ + ": Succeeded.  successfully completed.";
        as_.setSucceeded(result_);  /* action server's status is changed into "Succeeded". */
        return;
    }

}


void ToroboJointTrajectoryController::actionCallback()
{
    /*****************************************************************
      This callback is called by action server when a user send a goal.
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
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = as_.acceptNewGoal(); /* action server's status is changed into "Active". */
    pub_.publish(goal->trajectory);

    // Set Monitoring Value
    goal_time_ = ros::Time::now() + goal->trajectory.points[goal->trajectory.points.size()-1].time_from_start;
    goal_time_tolerance_ = goal->goal_time_tolerance + GOAL_TIME_TOLERANCE_MARGIN;
    for(int i=0; i<goal->trajectory.joint_names.size(); ++i)
    {
        jointgoal_map_[goal->trajectory.joint_names[i]] = goal->trajectory.points[goal->trajectory.points.size()-1].positions[i];
        tolerance_map_[goal->trajectory.joint_names[i]] = GOAL_TOLERANCE_MARGIN; // tentatively ignoring tolerance in action message
    }

}

/**
 * @file  MoveHomePositionAction.cpp
 * @brief Move home position action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "MoveHomePositionAction.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace std;

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

MoveHomePositionActionServer::MoveHomePositionActionServer(const ros::NodeHandle &node, const std::string nameSpace)
    : m_node(node),
        m_actionServer(m_node, nameSpace + "/move_home_position",
        boost::bind(&MoveHomePositionActionServer::ActionCallback, this, _1), false)
{
    m_node.param<double>("rateHz", m_rateHz, 10.0);
    m_node.param<double>("timeoutSec", m_timeoutSec, 5.0);
    m_homePosition = GetHomePositionFromRosParam();
    m_actionServer.start();
}

MoveHomePositionActionServer::~MoveHomePositionActionServer()
{
}

void MoveHomePositionActionServer::ActionCallback(const torobo_msgs::MoveHomePositionGoalConstPtr &goal)
{
    torobo_msgs::MoveHomePositionFeedback feedback;
    torobo_msgs::MoveHomePositionResult result;
    ros::Time lastTime = ros::Time::now();
    ros::Time currentTime = lastTime;

    ROS_INFO("Received goal: TransitionTime:%f", goal->transitionTime);

    try
    {
        double transitionTime = goal->transitionTime;
        trajectory_msgs::JointTrajectory jointTrajectory = GenerateJointTrajectoryFromJointPosition(m_homePosition, transitionTime);

        ROS_INFO("Start follow joint trajectory action");
        bool ret = CallFollowJointTrajectoryAction(jointTrajectory);

        while(true)
        {
            ros::spinOnce();

            if(m_actionServer.isPreemptRequested() || !ros::ok())
            {
                result.errorCode = result.PREEMPTED;
                m_actionServer.setPreempted(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                return;
            }

            currentTime = ros::Time::now();
            ros::Duration d((currentTime - lastTime).toSec());
            feedback.actual.time_from_start = d;

            ROS_INFO("Duration: %lf", d.toSec());

            if(d.toSec() >= goal->transitionTime)
            {
                result.errorCode = result.SUCCESSFUL;
                m_actionServer.setSucceeded(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setSucceeded ");
                return;
            }
            ros::Rate(m_rateHz).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result.errorCode = result.FAILURE;
        ROS_ERROR_STREAM(e.what());
        m_actionServer.setAborted(result);
        ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
    }
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
MoveHomePositionActionServer::JointPosition_t MoveHomePositionActionServer::GetHomePositionFromRosParam()
{
    // Get joint_names
    const string jointsParamName = m_node.getNamespace() + "/joints";
    if(!m_node.hasParam(jointsParamName))
    {
        throw std::runtime_error("Not found [" + jointsParamName + "]");
    }
    vector<string> jointNames;
    m_node.getParam(jointsParamName, jointNames);

    JointPosition_t homePosition;
    for(int i = 0; i < jointNames.size(); i++)
    {
        double value = 1.5;
        m_node.getParam(m_node.getNamespace() + "/home_position/" + jointNames[i], value);
        homePosition.jointNames.push_back(jointNames[i]);
        homePosition.positions.push_back(value);
    }

    return homePosition;
}

bool MoveHomePositionActionServer::CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory)
{
    string actionName = ros::this_node::getNamespace() + "/follow_joint_trajectory";
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(actionName);
    ac.waitForServer(ros::Duration(m_timeoutSec));

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = jointTrajectory;
    ac.sendGoal(goal);

    return true;
}

trajectory_msgs::JointTrajectory MoveHomePositionActionServer::GenerateJointTrajectoryFromJointPosition(const JointPosition_t jointPosition, const double transitionTime) const
{
    // Make JointTrajectoryPoint
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = jointPosition.positions;
    for(int i = 0; i < jointPosition.jointNames.size(); i++)
    {
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
        point.effort.push_back(0.0);
    }
    point.time_from_start = ros::Duration(transitionTime);

    // Make JointTrajectory
    trajectory_msgs::JointTrajectory jointTrajectory;
    jointTrajectory.header.stamp = ros::Time::now();
    jointTrajectory.joint_names = jointPosition.jointNames;
    jointTrajectory.points.push_back(point);

    return jointTrajectory;
}

}

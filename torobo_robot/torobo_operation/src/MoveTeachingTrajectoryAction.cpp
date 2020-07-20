/**
 * @file  MoveTeachingTrajectoryAction.cpp
 * @brief Move teaching trajectory action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "MoveTeachingTrajectoryAction.h"
#include "torobo_msgs/GetTeachingTrajectory.h"
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

MoveTeachingTrajectoryActionServer::MoveTeachingTrajectoryActionServer(const ros::NodeHandle &node, const std::string nameSpace)
    : m_node(node),
        m_actionServer(m_node, nameSpace + "/move_teaching_trajectory",
        boost::bind(&MoveTeachingTrajectoryActionServer::ActionCallback, this, _1), false)
{
    m_node.param<double>("rateHz", m_rateHz, 10.0);
    m_node.param<double>("timeoutSec", m_timeoutSec, 5.0);
    m_actionServer.start();
}

MoveTeachingTrajectoryActionServer::~MoveTeachingTrajectoryActionServer()
{
}

void MoveTeachingTrajectoryActionServer::ActionCallback(const torobo_msgs::MoveTeachingTrajectoryGoalConstPtr &goal)
{
    torobo_msgs::MoveTeachingTrajectoryFeedback feedback;
    torobo_msgs::MoveTeachingTrajectoryResult result;
    ros::Time lastTime = ros::Time::now();
    ros::Time currentTime = lastTime;

    ROS_INFO("Received goal: Traj Name:%s", goal->teachingTrajectoryName.c_str());

    try
    {
        trajectory_msgs::JointTrajectory jointTrajectory = CallGetTeachingTrajectoryService(goal->teachingTrajectoryName);
        double endTime = jointTrajectory.points.back().time_from_start.toSec();
        jointTrajectory.joint_names = GetJointNames();
        ROS_INFO("Traj length:%d, Traj end time:%lf", (int)jointTrajectory.points.size(), endTime);

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

            if(d.toSec() >= endTime)
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
trajectory_msgs::JointTrajectory MoveTeachingTrajectoryActionServer::CallGetTeachingTrajectoryService(const std::string teachingTrajectoryName)
{
    ros::ServiceClient client = m_node.serviceClient<torobo_msgs::GetTeachingTrajectory>("get_teaching_trajectory");
    torobo_msgs::GetTeachingTrajectory srv;
    srv.request.teachingTrajectoryName = teachingTrajectoryName;
    bool ret = client.call(srv);
    if(!ret || !(srv.response.success))
    {
        throw std::runtime_error("Not found teaching trajectory");
    }
    return srv.response.trajectory;
}

bool MoveTeachingTrajectoryActionServer::CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory)
{
    string actionName = ros::this_node::getNamespace() + "/follow_joint_trajectory";
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(actionName);
    ac.waitForServer(ros::Duration(m_timeoutSec));

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = jointTrajectory;
    ac.sendGoal(goal);

    return true;
}

std::vector<std::string> MoveTeachingTrajectoryActionServer::GetJointNames() const
{
    // Get joint_names
    const string jointsParamName = ros::this_node::getNamespace() + "/joints";
    if(!m_node.hasParam(jointsParamName))
    {
        throw std::runtime_error("Not found [" + jointsParamName + "]");
    }
    vector<string> jointNames;
    m_node.getParam(jointsParamName, jointNames);

    return jointNames;
}

}
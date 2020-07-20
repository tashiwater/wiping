/**
 * @file  MoveTeachingPointAction.cpp
 * @brief Move teaching point action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "MoveTeachingPointAction.h"
#include "torobo_msgs/GetTeachingPoint.h"
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

MoveTeachingPointActionServer::MoveTeachingPointActionServer(const ros::NodeHandle &node, const std::string nameSpace)
    : m_node(node),
        m_actionServer(m_node, nameSpace + "/move_teaching_point",
        boost::bind(&MoveTeachingPointActionServer::ActionCallback, this, _1), false)
{
    m_node.param<double>("rateHz", m_rateHz, 10.0);
    m_node.param<double>("timeoutSec", m_timeoutSec, 5.0);
    m_actionServer.start();
}

MoveTeachingPointActionServer::~MoveTeachingPointActionServer()
{
}

void MoveTeachingPointActionServer::ActionCallback(const torobo_msgs::MoveTeachingPointGoalConstPtr &goal)
{
    torobo_msgs::MoveTeachingPointFeedback feedback;
    torobo_msgs::MoveTeachingPointResult result;
    ros::Time lastTime = ros::Time::now();
    ros::Time currentTime = lastTime;

    ROS_INFO("Received goal: TP Name:%s, TransitionTime:%f", goal->teachingPointName.c_str(), goal->transitionTime);

    try
    {
        vector<double> positions = CallGetTeachingPointService(goal->teachingPointName);
        double transitionTime = goal->transitionTime;
        trajectory_msgs::JointTrajectory jointTrajectory = GenerateJointTrajectoryFromPositions(positions, transitionTime);

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
std::vector<double> MoveTeachingPointActionServer::CallGetTeachingPointService(const std::string teachingPointName)
{
    ros::ServiceClient client = m_node.serviceClient<torobo_msgs::GetTeachingPoint>("get_teaching_point");
    torobo_msgs::GetTeachingPoint srv;
    srv.request.teachingPointName = teachingPointName;
    bool ret = client.call(srv);
    if(!ret || !(srv.response.success))
    {
        throw std::runtime_error("Not found teaching point");
    }
    std::string str;
    for(int i = 0; i < srv.response.point.positions.size(); i++)
    {
        str += (to_string(srv.response.point.positions[i]) + ", ");
    }
    ROS_INFO("TP Pose: [%s]", str.c_str());
    return srv.response.point.positions;
}

bool MoveTeachingPointActionServer::CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory)
{
    string actionName = ros::this_node::getNamespace() + "/follow_joint_trajectory";
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(actionName);
    ac.waitForServer(ros::Duration(m_timeoutSec));

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = jointTrajectory;
    ac.sendGoal(goal);

    return true;
}

trajectory_msgs::JointTrajectory MoveTeachingPointActionServer::GenerateJointTrajectoryFromPositions(const std::vector<double> positions, const double transitionTime) const
{
    // Get joint_names
    const string jointsParamName = ros::this_node::getNamespace() + "/joints";
    if(!m_node.hasParam(jointsParamName))
    {
        throw std::runtime_error("Not found [" + jointsParamName + "]");
    }
    vector<string> jointNames;
    m_node.getParam(jointsParamName, jointNames);

    // Make JointTrajectoryPoint from teaching point
    trajectory_msgs::JointTrajectoryPoint point;
    for(int i = 0; i < jointNames.size(); i++)
    {
        point.positions.push_back(positions[i]);
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
        point.effort.push_back(0.0);
    }
    point.time_from_start = ros::Duration(transitionTime);

    // Make JointTrajectory
    trajectory_msgs::JointTrajectory jointTrajectory;
    jointTrajectory.header.stamp = ros::Time::now();
    jointTrajectory.joint_names = jointNames;
    jointTrajectory.points.push_back(point);

    return jointTrajectory;
}

}
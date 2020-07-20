/**
 * @file  MoveTeachingPointAction.h
 * @brief Move teaching point action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __MOVE_TEACHING_POINT_ACTION_H__
#define __MOVE_TEACHING_POINT_ACTION_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "torobo_msgs/MoveTeachingPointAction.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

class MoveTeachingPointActionServer
{
public:
    MoveTeachingPointActionServer(const ros::NodeHandle &node, const std::string nameSpace);
    virtual ~MoveTeachingPointActionServer();

    void ActionCallback(const torobo_msgs::MoveTeachingPointGoalConstPtr &goal);

protected:
    std::vector<double> CallGetTeachingPointService(const std::string teachingPointName);
    bool CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory);
    trajectory_msgs::JointTrajectory GenerateJointTrajectoryFromPositions(const std::vector<double> positions, const double transitionTime) const;

private:
    ros::NodeHandle m_node;
    actionlib::SimpleActionServer<torobo_msgs::MoveTeachingPointAction> m_actionServer;

    double m_rateHz;
    double m_timeoutSec;
};

}

#endif

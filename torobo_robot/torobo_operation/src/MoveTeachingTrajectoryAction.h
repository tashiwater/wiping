/**
 * @file  MoveTeachingTrajectoryAction.h
 * @brief Move teaching trajectory action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __MOVE_TEACHING_TRAJECTORY_ACTION_H__
#define __MOVE_TEACHING_TRAJECTORY_ACTION_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "torobo_msgs/MoveTeachingTrajectoryAction.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

class MoveTeachingTrajectoryActionServer
{
public:
    MoveTeachingTrajectoryActionServer(const ros::NodeHandle &node, const std::string nameSpace);
    virtual ~MoveTeachingTrajectoryActionServer();

    void ActionCallback(const torobo_msgs::MoveTeachingTrajectoryGoalConstPtr &goal);

protected:
    trajectory_msgs::JointTrajectory CallGetTeachingTrajectoryService(const std::string teachingTrajectoryName);
    bool CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory);
    std::vector<std::string> GetJointNames() const;

private:
    ros::NodeHandle m_node;
    actionlib::SimpleActionServer<torobo_msgs::MoveTeachingTrajectoryAction> m_actionServer;

    double m_rateHz;
    double m_timeoutSec;
};

}

#endif

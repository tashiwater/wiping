/**
 * @file  MoveHomePositionAction.h
 * @brief Move home position action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __MOVE_HOME_POSITION_ACTION_H__
#define __MOVE_HOME_POSITION_ACTION_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "torobo_msgs/MoveHomePositionAction.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

class MoveHomePositionActionServer
{
public:
    typedef struct
    {
        std::vector<std::string> jointNames;
        std::vector<double> positions;
    }JointPosition_t;

    MoveHomePositionActionServer(const ros::NodeHandle &node, const std::string nameSpace);
    virtual ~MoveHomePositionActionServer();

    void ActionCallback(const torobo_msgs::MoveHomePositionGoalConstPtr &goal);

protected:
    MoveHomePositionActionServer::JointPosition_t GetHomePositionFromRosParam();
    bool CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory);
    trajectory_msgs::JointTrajectory GenerateJointTrajectoryFromJointPosition(const JointPosition_t jointPosition, const double transitionTime) const;

private:
    ros::NodeHandle m_node;
    actionlib::SimpleActionServer<torobo_msgs::MoveHomePositionAction> m_actionServer;

    double m_rateHz;
    double m_timeoutSec;
    JointPosition_t m_homePosition;
};

}

#endif

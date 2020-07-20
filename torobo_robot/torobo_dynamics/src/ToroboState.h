/**
 * @file  ToroboState.h
 * @brief Torobo state class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_STATE_H__
#define __TOROBO_STATE_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <string>
#include <map>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "torobo_msgs/ToroboJointState.h"
#include "torobo_msgs/ToroboDynamics.h"

namespace torobo
{

class JointState;

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboState
{
public:
    ToroboState(ros::NodeHandle& node, std::map<std::string, int> nameJointsNumMap, bool sim);
    virtual ~ToroboState();

    const JointState* GetToroboJointState(std::string name) const;
    void Publish();

protected:
    void InitializeToroboJointStateMap(const std::string prefix, const int jointsNum);
    void UpdateJointState(const sensor_msgs::JointState::ConstPtr& msg);

    ros::NodeHandle& m_node;
    std::map<std::string, ros::Subscriber> m_torobo_state_sub;
    std::map<std::string, ros::Publisher> m_torobo_state_pub;
    ros::Subscriber m_joint_state_sub;
    std::map<std::string, ros::Publisher> m_torobo_dynamics_pub;

    std::map<std::string, int> m_nameJointsNumMap;
    std::map<std::string, JointState*> m_state;
    std::map<std::string, torobo_msgs::ToroboJointState::Ptr> m_toroboJointStateMap;

    ros::Time last_time_;
    std::map<std::string, double> last_position_;
    std::map<std::string, double> last_velocity_;

    std::map<std::string, torobo_msgs::ToroboDynamics> m_dynamics;
    bool sim_;

private:
};

}

#endif

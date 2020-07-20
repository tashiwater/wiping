/**
 * @file  JointState.h
 * @brief Joint state class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __JOINT_STATE_H__
#define __JOINT_STATE_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <vector>
#include <string>
#include "torobo_msgs/ToroboJointState.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class JointState
{
public:
    JointState(int jointsNum, std::string name="");
    virtual ~JointState();

    int GetJointsNum();
    void UpdateState(const torobo_msgs::ToroboJointState::ConstPtr& msg);

    std::vector<std::string> m_joint_names;
    std::vector<double> m_current;
    std::vector<double> m_position;
    std::vector<double> m_velocity;
    std::vector<double> m_refVelocity;
    std::vector<double> m_acceleration;
    std::vector<double> m_refAcceleration;
    std::vector<double> m_effort;
protected:
    int m_jointsNum;
    std::string m_name;

private:
};

}

#endif

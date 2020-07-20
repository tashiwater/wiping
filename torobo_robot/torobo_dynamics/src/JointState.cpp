/**
 * @file  JointState.h
 * @brief Joint state class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "JointState.h"

// #define MEASURE_TIME
#ifdef MEASURE_TIME
#include <chrono>
#include <iostream>
static std::chrono::time_point<std::chrono::system_clock> t0;
#endif

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
JointState::JointState(int jointsNum, std::string name)
{
    m_jointsNum = jointsNum;
    m_name = name;

    for(int i = 0; i < m_jointsNum; i++)
    {
        m_joint_names.push_back("");
        m_position.push_back(0.0);
        m_velocity.push_back(0.0);
        m_refVelocity.push_back(0.0);
        m_acceleration.push_back(0.0);
        m_refAcceleration.push_back(0.0);
        m_effort.push_back(0.0);
        m_current.push_back(0.0);
    }
}

JointState::~JointState()
{
}

int JointState::GetJointsNum()
{
    return m_jointsNum;
}

void JointState::UpdateState(const torobo_msgs::ToroboJointState::ConstPtr& msg)
{
    int size = (int)msg->name.size();
    if(size > m_jointsNum)
    {
        size = m_jointsNum;
    }
    for(int i = 0; i < size; i++)
    {
        m_joint_names[i] = msg->name[i];
        m_current[i] = msg->current[i];
        m_position[i] = msg->position[i];
#if 1
        m_velocity[i] = msg->outConvInVelocity[i];
        m_acceleration[i] = msg->outConvInAcceleration[i];
#else
        m_velocity[i] = msg->velocity[i];
        m_acceleration[i] = msg->acceleration[i];
#endif
        m_refVelocity[i] = msg->refVelocity[i];
        m_refAcceleration[i] = msg->refAcceleration[i];
        m_effort[i] = msg->effort[i];
    }

#ifdef MEASURE_TIME
    auto t = chrono::system_clock::now();
    auto usec = chrono::duration_cast<chrono::microseconds>(t - t0).count();
    t0 = t;
    std::cout << "[torobo_dynamics][" << m_name.c_str() << "]Time: " << usec << std::endl;
#endif
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/

}

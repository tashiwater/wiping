/**
 * @file  JointTrajectoryControllerStatePublisher.cpp
 * @brief Joint trajectory controller state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "JointTrajectoryControllerStatePublisher.h"

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
JointTrajectoryControllerStatePublisher::JointTrajectoryControllerStatePublisher(ros::NodeHandle &node,
                                         const std::string ns,
                                         const std::map<std::string, int> jointsNameIdMap,
                                         const JointTrajectoryControllerStatePublisher::Coef coef,
                                         const int queueSize)
    : AbstractPublisher(node, ns, jointsNameIdMap),
      m_coef(coef)
{
    Initialize(queueSize);
}

JointTrajectoryControllerStatePublisher::~JointTrajectoryControllerStatePublisher()
{
}

void JointTrajectoryControllerStatePublisher::Publish(const RecvPacket& packet, const ros::Time rosTimeStamp)
{
    m_jointTrajectoryControllerState.header.stamp = rosTimeStamp;
    int idx = 0;
    for(auto itr = m_jointsNameIdMap.begin(); itr != m_jointsNameIdMap.end(); ++itr)
    {
        const int i = itr->second;
        const double refpos = packet.m_joint[i].refPosition * m_coef.position;
        const double refvel = packet.m_joint[i].refVelocity * m_coef.velocity;
        const double refacc = packet.m_joint[i].refAcceleration * m_coef.acceleration;
        const double refeft = packet.m_joint[i].refEffort * m_coef.effort;
        const double curpos = packet.m_joint[i].position * m_coef.position;
        const double curvel = packet.m_joint[i].velocity * m_coef.velocity;
        const double curacc = packet.m_joint[i].acceleration * m_coef.acceleration;
        const double cureft = packet.m_joint[i].effort * m_coef.effort;
        const int32_t sec = (int32_t)packet.m_preData.duration;
        const int32_t nsec = (int32_t)(packet.m_preData.duration * 1000000000 - (int32_t)packet.m_preData.duration);
        m_jointTrajectoryControllerState.desired.positions[idx] = refpos;
        m_jointTrajectoryControllerState.desired.velocities[idx] = refvel;
        m_jointTrajectoryControllerState.desired.accelerations[idx] = refacc;
        m_jointTrajectoryControllerState.desired.effort[idx] = refeft;
        m_jointTrajectoryControllerState.desired.time_from_start.sec = sec;
        m_jointTrajectoryControllerState.desired.time_from_start.nsec = nsec;
        m_jointTrajectoryControllerState.actual.positions[idx] = curpos;
        m_jointTrajectoryControllerState.actual.velocities[idx] = curvel;
        m_jointTrajectoryControllerState.actual.accelerations[idx] = curacc;
        m_jointTrajectoryControllerState.actual.effort[idx] = cureft;
        m_jointTrajectoryControllerState.actual.time_from_start.sec = sec;
        m_jointTrajectoryControllerState.actual.time_from_start.nsec = nsec;
        m_jointTrajectoryControllerState.error.positions[idx] = refpos - curpos;
        m_jointTrajectoryControllerState.error.velocities[idx] = refvel - curvel;
        m_jointTrajectoryControllerState.error.accelerations[idx] = refacc - curacc;
        m_jointTrajectoryControllerState.error.effort[idx] = refeft - cureft;
        m_jointTrajectoryControllerState.error.time_from_start.sec = sec;
        m_jointTrajectoryControllerState.error.time_from_start.nsec = nsec;
        idx++;
    }
    m_pub.publish(m_jointTrajectoryControllerState);
}


/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void JointTrajectoryControllerStatePublisher::Initialize(const int queueSize)
{
    m_jointTrajectoryControllerState.header.stamp = ros::Time::now();
    for(auto itr = m_jointsNameIdMap.begin(); itr != m_jointsNameIdMap.end(); ++itr)
    {
        m_jointTrajectoryControllerState.joint_names.push_back(itr->first);
        m_jointTrajectoryControllerState.desired.positions.push_back(0.0);
        m_jointTrajectoryControllerState.desired.velocities.push_back(0.0);
        m_jointTrajectoryControllerState.desired.accelerations.push_back(0.0);
        m_jointTrajectoryControllerState.desired.effort.push_back(0.0);
        m_jointTrajectoryControllerState.actual.positions.push_back(0.0);
        m_jointTrajectoryControllerState.actual.velocities.push_back(0.0);
        m_jointTrajectoryControllerState.actual.accelerations.push_back(0.0);
        m_jointTrajectoryControllerState.actual.effort.push_back(0.0);
        m_jointTrajectoryControllerState.error.positions.push_back(0.0);
        m_jointTrajectoryControllerState.error.velocities.push_back(0.0);
        m_jointTrajectoryControllerState.error.accelerations.push_back(0.0);
        m_jointTrajectoryControllerState.error.effort.push_back(0.0);
    }
    m_pub = m_node.advertise<control_msgs::JointTrajectoryControllerState>(m_ns + "/state", queueSize, true);
}

}

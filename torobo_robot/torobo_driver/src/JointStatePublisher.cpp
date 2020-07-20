/**
 * @file  JointStatePublisher.cpp
 * @brief Joint state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "JointStatePublisher.h"

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
JointStatePublisher::JointStatePublisher(ros::NodeHandle &node,
                                         const std::string ns,
                                         const std::map<std::string, int> jointsNameIdMap,
                                         const JointStatePublisher::Coef coef,
                                         const int queueSize)
    : AbstractPublisher(node, ns, jointsNameIdMap),
      m_coef(coef)
{
    Initialize(queueSize);
}

JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::Publish(const RecvPacket& packet, const ros::Time rosTimeStamp)
{
    m_jointState.header.stamp = rosTimeStamp;
    int idx = 0;
    for(auto itr = m_jointsNameIdMap.begin(); itr != m_jointsNameIdMap.end(); ++itr)
    {
        const int i = itr->second;
        m_jointState.position[idx] = packet.m_joint[i].position * m_coef.position;
        m_jointState.velocity[idx] = packet.m_joint[i].velocity * m_coef.velocity;
        m_jointState.effort[idx] = packet.m_joint[i].effort * m_coef.effort;
        idx++;
    }
    m_pub.publish(m_jointState);
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void JointStatePublisher::Initialize(const int queueSize)
{
    m_jointState.header.stamp = ros::Time::now();
    for(auto itr = m_jointsNameIdMap.begin(); itr != m_jointsNameIdMap.end(); ++itr)
    {
        m_jointState.name.push_back(itr->first);
        m_jointState.position.push_back(0.0);
        m_jointState.velocity.push_back(0.0);
        m_jointState.effort.push_back(0.0);
    }
    m_pub = m_node.advertise<sensor_msgs::JointState>(m_ns + "/joint_state", queueSize, true);
}

}

/**
 * @file  ToroboJointStatePublisher.cpp
 * @brief Joint trajectory controller state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "ToroboJointStatePublisher.h"

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboJointStatePublisher::ToroboJointStatePublisher(ros::NodeHandle &node,
                                         const std::string ns,
                                         const std::map<std::string, int> jointsNameIdMap,
                                         const ToroboJointStatePublisher::Coef coef,
                                         const int queueSize)
    : AbstractPublisher(node, ns, jointsNameIdMap),
      m_coef(coef)
{
    Initialize(queueSize);
}

ToroboJointStatePublisher::~ToroboJointStatePublisher()
{
}

void ToroboJointStatePublisher::Publish(const RecvPacket& packet, const ros::Time rosTimeStamp)
{
    m_toroboJointState.header.stamp = rosTimeStamp;
    m_toroboJointState.timeStamp = packet.m_preData.timeStamp;
    m_toroboJointState.hostTimeStamp = packet.m_preData.hostTimeStamp;

    int idx = 0;
    for(auto itr = m_jointsNameIdMap.begin(); itr != m_jointsNameIdMap.end(); ++itr)
    {
        const int i = itr->second;
        m_toroboJointState.type[idx] = packet.m_joint[i].type;
        m_toroboJointState.comStatus[idx] = packet.m_joint[i].comStatus;
        m_toroboJointState.systemMode[idx] = packet.m_joint[i].systemMode;
        m_toroboJointState.ctrlMode[idx] = packet.m_joint[i].ctrlMode;
        m_toroboJointState.errorWarningStatus[idx] = packet.m_joint[i].ewStatus;
        m_toroboJointState.trjStatus[idx] = packet.m_joint[i].trjStatus;
        m_toroboJointState.trjViaRemain[idx] = packet.m_joint[i].trjViaRemain;
        m_toroboJointState.refCurrent[idx] = packet.m_joint[i].refCurrent;
        m_toroboJointState.refPosition[idx] = packet.m_joint[i].refPosition * m_coef.position;
        m_toroboJointState.refVelocity[idx] = packet.m_joint[i].refVelocity * m_coef.velocity;
        m_toroboJointState.refAcceleration[idx] = packet.m_joint[i].refAcceleration * m_coef.acceleration;
        m_toroboJointState.refEffort[idx] = packet.m_joint[i].refEffort * m_coef.effort;
        m_toroboJointState.current[idx] = packet.m_joint[i].current;
        m_toroboJointState.position[idx] = packet.m_joint[i].position * m_coef.position;
        m_toroboJointState.velocity[idx] = packet.m_joint[i].velocity * m_coef.velocity;
        m_toroboJointState.acceleration[idx] = packet.m_joint[i].acceleration * m_coef.acceleration;
        m_toroboJointState.outConvInVelocity[idx] = packet.m_joint[i].outConvInVelocity * m_coef.velocity;
        m_toroboJointState.outConvInAcceleration[idx] = packet.m_joint[i].outConvInAcceleration * m_coef.acceleration;
        m_toroboJointState.effort[idx] = packet.m_joint[i].effort * m_coef.effort;
        m_toroboJointState.temperature[idx] = packet.m_joint[i].temperature;
        m_toroboJointState.general_0[idx] = packet.m_joint[i].general[0];
        m_toroboJointState.general_1[idx] = packet.m_joint[i].general[1];
        m_toroboJointState.general_2[idx] = packet.m_joint[i].general[2];
        m_toroboJointState.general_3[idx] = packet.m_joint[i].general[3];
        idx++;
    }
    m_pub.publish(m_toroboJointState);
}




/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboJointStatePublisher::Initialize(const int queueSize)
{
    m_toroboJointState.header.stamp = ros::Time::now();
    m_toroboJointState.timeStamp = 0;
    m_toroboJointState.hostTimeStamp = 0;
    for(auto itr = m_jointsNameIdMap.begin(); itr != m_jointsNameIdMap.end(); ++itr)
    {
        m_toroboJointState.name.push_back(itr->first);
        m_toroboJointState.type.push_back(0);
        m_toroboJointState.comStatus.push_back(0);
        m_toroboJointState.systemMode.push_back(0);
        m_toroboJointState.ctrlMode.push_back(0);
        m_toroboJointState.errorWarningStatus.push_back(0);
        m_toroboJointState.trjStatus.push_back(0);
        m_toroboJointState.trjViaRemain.push_back(0);
        m_toroboJointState.refCurrent.push_back(0.0);
        m_toroboJointState.refPosition.push_back(0.0);
        m_toroboJointState.refVelocity.push_back(0.0);
        m_toroboJointState.refAcceleration.push_back(0.0);
        m_toroboJointState.refEffort.push_back(0.0);
        m_toroboJointState.current.push_back(0.0);
        m_toroboJointState.position.push_back(0.0);
        m_toroboJointState.velocity.push_back(0.0);
        m_toroboJointState.acceleration.push_back(0.0);
        m_toroboJointState.outConvInVelocity.push_back(0.0);
        m_toroboJointState.outConvInAcceleration.push_back(0.0);
        m_toroboJointState.effort.push_back(0.0);
        m_toroboJointState.temperature.push_back(0.0);
        m_toroboJointState.general_0.push_back(0.0);
        m_toroboJointState.general_1.push_back(0.0);
        m_toroboJointState.general_2.push_back(0.0);
        m_toroboJointState.general_3.push_back(0.0);
    }
    m_pub = m_node.advertise<torobo_msgs::ToroboJointState>(m_ns + "/torobo_joint_state", queueSize, true);
}


}

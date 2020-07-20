/**
 * @file  ToroboState.h
 * @brief Torobo state class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include "ToroboState.h"
#include "JointState.h"

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboState::ToroboState(ros::NodeHandle& node, std::map<std::string, int> nameJointsNumMap, bool sim)
 : m_node(node)
{
    m_nameJointsNumMap = nameJointsNumMap;
    sim_ = sim;

    for(auto itr = m_nameJointsNumMap.begin(); itr != m_nameJointsNumMap.end(); ++itr)
    {
        string name = itr->first;
        int jointsNum = itr->second;

        // Generate state subscriber
        JointState* state = new JointState(jointsNum, name);
#if 0
        ros::Subscriber sub = m_node.subscribe(m_node.getNamespace() + "/" + name + "_controller/torobo_joint_state", 1, &JointState::UpdateState, state);
#else
        ros::Subscriber sub = m_node.subscribe(m_node.getNamespace() + "/" + name + "_controller/torobo_joint_state", 1, &JointState::UpdateState, state, ros::TransportHints().reliable().tcpNoDelay(true));
#endif
        m_state.insert(make_pair(name, state));
        m_torobo_state_sub.insert(make_pair(name, sub));
    }
    if(sim_)
    {
        m_joint_state_sub = m_node.subscribe(m_node.getNamespace() + "/joint_states" , 1, &ToroboState::UpdateJointState, this);
        for(auto itr = m_nameJointsNumMap.begin(); itr != m_nameJointsNumMap.end(); ++itr)
        {
            string name = itr->first;
            int jointsNum = itr->second;
            InitializeToroboJointStateMap(name, jointsNum);
            ros::Publisher pub  = m_node.advertise<torobo_msgs::ToroboJointState>(m_node.getNamespace() + "/" + name + "_controller/torobo_joint_state", 1);
            m_torobo_state_pub.insert(make_pair(name, pub));
        }
    }
}

ToroboState::~ToroboState()
{
    for(auto itr = m_state.begin(); itr != m_state.end(); ++itr)
    {
        if(itr->second != NULL)
        {
            delete itr->second;
            itr->second = NULL;
        }
    }
}

const JointState* ToroboState::GetToroboJointState(std::string name) const
{
    if(m_state.count(name) > 0)
    {
        return m_state.at(name);
    }
    return NULL;
}

void ToroboState::Publish()
{
    if(!sim_)
    {
        return;
    }
    for(auto itr = m_toroboJointStateMap.begin(); itr != m_toroboJointStateMap.end(); ++itr)
    {
        string name = itr->first;
        if(m_torobo_state_pub.count(name) == 0)
        {
            continue;
        }
        if(itr->second->name[0] == "")
        {
            continue;
        }

        itr->second->header.stamp = ros::Time::now();
        m_torobo_state_pub.at(name).publish(itr->second);
    }
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboState::InitializeToroboJointStateMap(const std::string prefix, const int jointsNum)
{
    //torobo_msgs::ToroboJointState toroboJointState;
    torobo_msgs::ToroboJointState::Ptr toroboJointState(new torobo_msgs::ToroboJointState());
    for(int i = 0; i < jointsNum; i++)
    {
        toroboJointState->name.push_back("");
        toroboJointState->type.push_back(0);
        toroboJointState->comStatus.push_back(0);
        toroboJointState->systemMode.push_back(0);
        toroboJointState->ctrlMode.push_back(0);
        toroboJointState->errorWarningStatus.push_back(0);
        toroboJointState->trjStatus.push_back(0);
        toroboJointState->trjViaRemain.push_back(0);
        toroboJointState->refCurrent.push_back(0.0);
        toroboJointState->refPosition.push_back(0.0);
        toroboJointState->refVelocity.push_back(0.0);
        toroboJointState->refAcceleration.push_back(0.0);
        toroboJointState->refEffort.push_back(0.0);
        toroboJointState->current.push_back(0.0);
        toroboJointState->position.push_back(0.0);
        toroboJointState->velocity.push_back(0.0);
        toroboJointState->acceleration.push_back(0.0);
        toroboJointState->outConvInVelocity.push_back(0.0);
        toroboJointState->outConvInAcceleration.push_back(0.0);
        toroboJointState->effort.push_back(0.0);
        toroboJointState->temperature.push_back(0.0);
        toroboJointState->general_0.push_back(0.0);
        toroboJointState->general_1.push_back(0.0);
        toroboJointState->general_2.push_back(0.0);
        toroboJointState->general_3.push_back(0.0);
    }
    m_toroboJointStateMap.insert(make_pair(prefix, toroboJointState));
}

void ToroboState::UpdateJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
    //sensor_msgs::JointState joint_state = *msg;
    ros::Time t = ros::Time::now();
    ros::Duration d = t - last_time_;
    double dt = d.toSec();
    if(dt < 0.0)
    {
        return;
    }

    std::map<std::string, int> prefixJointCountMap;
    for(auto itr = m_toroboJointStateMap.begin(); itr != m_toroboJointStateMap.end(); ++itr)
    {
        string prefix = itr->first;
        prefixJointCountMap.insert(std::make_pair(prefix, 0));
    }

    //for(int i = 0; i < joint_state->name.size(); i++)
    for(int i = 0; i < msg->name.size(); i++)
    {
        //string name = joint_state->name[i];
        string name = msg->name[i];
        //double position = joint_state->position[i];
        double position = msg->position[i];

        for(auto itr = m_toroboJointStateMap.begin(); itr != m_toroboJointStateMap.end(); ++itr)
        {
            string prefix = itr->first;
            if (name.find(prefix) != std::string::npos)
            {
                const int idx = prefixJointCountMap[prefix];
                itr->second->name[idx] = name;
                itr->second->position[idx] = position;

                // estimate velocity & acceleration
                double velocity = 0.0;
                if(last_position_.count(name) > 0)
                {
                    velocity = (position - last_position_[name]) / dt;
                }

                double acceleration = 0.0;
                if(last_velocity_.count(name) > 0)
                {
                    acceleration = (velocity - last_velocity_[name]) / dt;
                }
                itr->second->velocity[idx] = velocity;
                itr->second->acceleration[idx] = acceleration;
                last_position_[name] = position;
                last_velocity_[name] = velocity;

                prefixJointCountMap[prefix]++;
            }
        }
    }
    last_time_ = t;
    Publish();
}

}

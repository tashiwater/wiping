/**
 * @file  ToroboDynamics.h
 * @brief Torobo Dynamics class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <vector>
#include "ToroboDynamics.h"
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
ToroboDynamics::ToroboDynamics(ros::NodeHandle& node, const std::map<std::string, int>& nameJointsNumMap)
 : m_node(node)
{
    m_nameJointsNumMap = nameJointsNumMap;

    model_ = new ToroboRbdlModel();

    for(auto itr = m_nameJointsNumMap.begin(); itr != m_nameJointsNumMap.end(); ++itr)
    {
        const string& name = itr->first;
        const int jointsNum = itr->second;

        // Generate dynamics publisher
        ros::Publisher pub  = m_node.advertise<torobo_msgs::ToroboDynamics>(m_node.getNamespace() + "/" + name + "_controller/torobo_dynamics", 1);
        torobo_msgs::ToroboDynamics dyna;
        InitializeToroboDynamics(dyna, name, jointsNum);
        m_dynamics.insert(make_pair(name, dyna));
        m_torobo_dynamics_pub.insert(make_pair(name, pub));

        //Generate ServiceServer
        ros::ServiceServer srv = m_node.advertiseService(m_node.getNamespace() + "/" + name + "_controller/set_payload_param", &ToroboDynamics::SetPayloadParamService,this);
        m_service.insert(make_pair(name, srv));
    }
}

ToroboDynamics::~ToroboDynamics()
{
    if(model_ != nullptr)
    {
        delete model_;
        model_ = nullptr;
    }
}

void ToroboDynamics::Update(const std::unique_ptr<ToroboState>& toroboState)
{
    // Update gravity compensation effort
    for(auto itr = m_nameJointsNumMap.begin(); itr != m_nameJointsNumMap.end(); ++itr)
    {
        const string& prefix = itr->first;
        const JointState* const jointState = toroboState->GetToroboJointState(prefix);

        // Calc gravity compensation effort
        const vector<double> zeroVec(jointState->m_joint_names.size(), 0.0);
        model_->UpdateQ(    jointState->m_joint_names, jointState->m_position);
        model_->UpdateQDot( jointState->m_joint_names, zeroVec);
        model_->UpdateQDDot(jointState->m_joint_names, zeroVec);
    }
    model_->CalcInverseDynamics();
    for(auto itr = m_nameJointsNumMap.begin(); itr != m_nameJointsNumMap.end(); ++itr)
    {
        const string& prefix = itr->first;
        SetGravityCompensationEffort(prefix, model_->GetTau(prefix, m_nameJointsNumMap[prefix]));
    }

    // Update cur dynamics effort & inertia matrix
    for(auto itr = m_nameJointsNumMap.begin(); itr != m_nameJointsNumMap.end(); ++itr)
    {
        const string& prefix = itr->first;
        const JointState* const jointState = toroboState->GetToroboJointState(prefix);

        // Calc gravity compensation effort
        model_->UpdateQDot( jointState->m_joint_names, jointState->m_velocity);
        model_->UpdateQDDot(jointState->m_joint_names, jointState->m_acceleration);
    }
    model_->CalcInverseDynamics();
    model_->CalcCompositeRigidBodyAlgorithm();
    for(auto itr = m_nameJointsNumMap.begin(); itr != m_nameJointsNumMap.end(); ++itr)
    {
        const string& prefix = itr->first;
        SetCurDynamicsEffort(prefix, model_->GetTau(prefix, m_nameJointsNumMap[prefix]));
        SetInertiaDiagonal(prefix, model_->GetInertiaDiagonal(prefix, m_nameJointsNumMap[prefix]));
    }

    // Update ref dynamics effort
    for(auto itr = m_nameJointsNumMap.begin(); itr != m_nameJointsNumMap.end(); ++itr)
    {
        const string& prefix = itr->first;
        const JointState* const jointState = toroboState->GetToroboJointState(prefix);

        // Calc gravity compensation effort
        model_->UpdateQDot( jointState->m_joint_names, jointState->m_refVelocity);
        model_->UpdateQDDot(jointState->m_joint_names, jointState->m_refAcceleration);
    }
    model_->CalcInverseDynamics();
    for(auto itr = m_nameJointsNumMap.begin(); itr != m_nameJointsNumMap.end(); ++itr)
    {
        const string& prefix = itr->first;
        SetRefDynamicsEffort(prefix, model_->GetTau(prefix, m_nameJointsNumMap[prefix]));
    }
}

void ToroboDynamics::Publish()
{
    for(auto itr = m_dynamics.begin(); itr != m_dynamics.end(); ++itr)
    {
        string name = itr->first;
        if(m_torobo_dynamics_pub.count(name) == 0)
        {
            continue;
        }
        itr->second.header.stamp = ros::Time::now();
        m_torobo_dynamics_pub.at(name).publish(itr->second);
    }
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboDynamics::InitializeToroboDynamics(torobo_msgs::ToroboDynamics& dyna, const std::string& name, const int jointsNum) const
{
    // Initialize parameters
    dyna.header = std_msgs::Header();
    dyna.header.stamp = ros::Time::now();
    for(int i = 0; i < jointsNum; i++)
    {
        string jointName = name +"/joint_" + to_string(i+1);
        dyna.name.push_back(jointName);
        dyna.gravity_compensation_effort.push_back(0.0);
        dyna.ref_dynamics_effort.push_back(0.0);
        dyna.cur_dynamics_effort.push_back(0.0);
        dyna.inertia_diagonal.push_back(0.0);
    }
}

void ToroboDynamics::SetRefDynamicsEffort(const std::string& name, const std::vector<double>& effort)
{
    size_t s = m_dynamics[name].ref_dynamics_effort.size();
    if(s != effort.size())
    {
        return;
    }
    for(int i = 0; i < s; i++)
    {
        m_dynamics[name].ref_dynamics_effort[i] = effort[i];
    }
}

void ToroboDynamics::SetCurDynamicsEffort(const std::string& name, const std::vector<double>& effort)
{
    size_t s = m_dynamics[name].cur_dynamics_effort.size();
    if(s != effort.size())
    {
        return;
    }
    for(int i = 0; i < s; i++)
    {
        m_dynamics[name].cur_dynamics_effort[i] = effort[i];
    }
}

void ToroboDynamics::SetInertiaDiagonal(const std::string& name, const std::vector<double>& inertiaDiagonal)
{
    size_t s = m_dynamics[name].inertia_diagonal.size();
    if(s != inertiaDiagonal.size())
    {
        return;
    }
    for(int i = 0; i < s; i++)
    {
        m_dynamics[name].inertia_diagonal[i] = inertiaDiagonal[i];
    }
}

void ToroboDynamics::SetGravityCompensationEffort(const std::string& name, const std::vector<double>& gravityCompEffort)
{
    size_t s = m_dynamics[name].gravity_compensation_effort.size();
    if(s != gravityCompEffort.size())
    {
        return;
    }
    for(int i = 0; i < s; i++)
    {
        m_dynamics[name].gravity_compensation_effort[i] = gravityCompEffort[i];
    }
}

bool ToroboDynamics::SetPayloadParamService(torobo_msgs::SetPayloadParam::Request &req , torobo_msgs::SetPayloadParam::Response &res)
{
    ROS_INFO("[ToroboDynamics:%s] SetPayloadParamService is called.", req.name.c_str());

    if(m_nameJointsNumMap.count(req.name) == 0)
    {
        ROS_ERROR("[ToroboDynamics:%s] Invalid name is given. Failed to add fixed body.", req.name.c_str());
        res.success = false;
        return false;
    }
    const int jointsNum = m_nameJointsNumMap.at(req.name);

    RigidBodyDynamics::Math::Vector3d com(0.0, 0.0, 0.0);
    if(req.com.size() == 3)
    {
        com = RigidBodyDynamics::Math::Vector3d(req.com[0], req.com[1], req.com[2]);
    }
    
    double mass = req.mass;
    RigidBodyDynamics::Math::Matrix3d inertiaMatrix;
    double defaultMomentOfInertia = 1.0e-8;

    if(req.inertiaElem.size() == 6) 
    {
    //    ROS_INFO("ixx:%f, ixy:%f, ixz:%f, iyy:%f, iyz:%f, izz:%f is set.",req.inertiaElem[0],req.inertiaElem[1],req.inertiaElem[2],req.inertiaElem[3],req.inertiaElem[4],req.inertiaElem[5]);
        inertiaMatrix << req.inertiaElem[0] , req.inertiaElem[1] , req.inertiaElem[2] ,
                         req.inertiaElem[1] , req.inertiaElem[3] , req.inertiaElem[4] , 
                         req.inertiaElem[2] , req.inertiaElem[4] , req.inertiaElem[5] ;
    }
    else
    {
    //    ROS_INFO("ixx:%f, ixy:%f, ixz:%f, iyy:%f, iyz:%f, izz:%f is set.",defaultMomentOfInertia, 0.0, 0.0, defaultMomentOfInertia, 0.0, defaultMomentOfInertia);
        inertiaMatrix << defaultMomentOfInertia, 0.0  , 0.0,
                         0.0    ,defaultMomentOfInertia, 0.0,
                         0.0    ,   0.0, defaultMomentOfInertia;
    }

    res.success = model_->AddFixedBody(req.name, jointsNum, mass , com,inertiaMatrix);
    if(!res.success)
    {
        ROS_ERROR("[ToroboDynamics:%s] Failed to add fixed body.", req.name.c_str());
        return false;
    }
    ROS_INFO("[ToroboDynamics:%s] Succeeded to add fixed body.", req.name.c_str());
    return true;
}

}

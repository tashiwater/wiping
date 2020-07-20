/**
 * @file  ToroboJointStateController.cpp
 * @brief ToroboJointStateController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <iostream>
#include "ToroboJointStateServer.h"


using namespace std;

/*----------------------------------------------------------------------
 Protected Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboJointStateServer::ToroboJointStateServer(ros::NodeHandle& node) : nh_(node)
{
    sub_ = nh_.subscribe(nh_.getNamespace() + "/joint_states", 1, &ToroboJointStateServer::sourceCallback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    pub_ = nh_.advertise<sensor_msgs::JointState>(nh_.getNamespace() + "/joint_state_server/joint_states", 1, this);
    joint_state_ = NULL;

    services_.push_back(nh_.advertiseService(nh_.getNamespace() + "/joint_state_server/get_joint_state", &ToroboJointStateServer::jointStateService, this));
    services_.push_back(nh_.advertiseService(nh_.getNamespace() + "/joint_state_server/get_torobo_joint_state", &ToroboJointStateServer::toroboJointStateService, this));
}

ToroboJointStateServer::~ToroboJointStateServer()
{
    sub_.shutdown();
    pub_.shutdown();
    for(auto itr = services_.begin(); itr != services_.end(); ++itr)
    {
        itr->shutdown();
    }
}

void ToroboJointStateServer::setRate(double rate)
{
    publish_rate_ = rate;
}

void ToroboJointStateServer::registerController(std::string controller_name)
{
    std::string topic_name = nh_.getNamespace() + "/" + controller_name + "/torobo_joint_state";
    sub_list_.push_back(nh_.subscribe(topic_name, 1, &ToroboJointStateServer::sourceToroboCallback, this, ros::TransportHints().reliable().tcpNoDelay(true)) ) ;
    torobojointstate_map_[nh_.resolveName(topic_name)] = NULL;
    pub_map_[nh_.resolveName(topic_name)] = nh_.advertise<torobo_msgs::ToroboJointState>(nh_.getNamespace() + "/joint_state_server/" + controller_name + "/torobo_joint_state", 1, this);
}

void ToroboJointStateServer::start()
{
    timer_ = nh_.createTimer(ros::Duration(1.0 / (double)publish_rate_), &ToroboJointStateServer::timerCallback, this);
}

/*----------------------------------------------------------------------
 Protected Method Implementations
 ----------------------------------------------------------------------*/
void ToroboJointStateServer::sourceCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state_ = msg;
}

void ToroboJointStateServer::sourceToroboCallback(const ros::MessageEvent<torobo_msgs::ToroboJointState const>& event)
{
    const ros::M_string& header = event.getConnectionHeader();
    std::string topic = header.at("topic");
    //std::cout << topic << std::endl;
    const torobo_msgs::ToroboJointState::ConstPtr& msg = event.getMessage();
    torobojointstate_map_[topic] = msg;
    //std::cout << msg << std::endl;
    //std::cout << msg->ctrlMode[0] << std::endl;
}

void ToroboJointStateServer::timerCallback(const ros::TimerEvent& e)
{
    if(!joint_state_)
    {
        return;
    }
    pub_.publish(joint_state_);

    for(auto itr = pub_map_.begin(); itr != pub_map_.end(); ++itr)
    {
        const torobo_msgs::ToroboJointState::ConstPtr& msg = torobojointstate_map_[itr->first];
        //std::cout << itr->first << std::endl;
        //std::cout << msg << std::endl;
        if(msg)
        {
            itr->second.publish(msg);
        }
    }
}

bool ToroboJointStateServer::jointStateService(torobo_msgs::GetJointState::Request &req, torobo_msgs::GetJointState::Response &res)
{
    if(joint_state_)
    {
        res.jointState = *joint_state_;
    }
    else
    {
        ROS_ERROR("joint_state data has not configured yet.");
        return false;
    }

    return true;
}

bool ToroboJointStateServer::toroboJointStateService(torobo_msgs::GetToroboJointState::Request &req, torobo_msgs::GetToroboJointState::Response &res)
{
    const string& controllerName = req.controllerName;
    for(auto itr = pub_map_.begin(); itr != pub_map_.end(); ++itr)
    {
        if (itr->first.find(controllerName) == std::string::npos)
        {
            continue;
        }
        if(!torobojointstate_map_[itr->first])
        {
            // ROS_ERROR("torobo joint_state data has not configured yet.");
            return false;
        }
        res.toroboJointState = *torobojointstate_map_[itr->first];
        return true;
    }
    ROS_ERROR("given controller name is not found.");
    return false;
}

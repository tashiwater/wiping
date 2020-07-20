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
#define _USE_MATH_DEFINES
#include <cmath>
#include "ToroboJointStateController.h"


using namespace std;

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboJointStateController::ToroboJointStateController(ros::NodeHandle& node, std::string name) : nh_(node), name_(name)
{
    pub_ = nh_.advertise<sensor_msgs::JointState>(nh_.getNamespace() + "/joint_states", 1, this);
}

ToroboJointStateController::~ToroboJointStateController()
{
    for (auto itr = sub_list_.begin(); itr != sub_list_.end(); ++itr)
    {
        itr->shutdown();
    }
    pub_.shutdown();
}

void ToroboJointStateController::registerController(std::string controller_name)
{
    std::string topic_name = nh_.getNamespace() + "/" + controller_name + "/joint_state";
    sub_list_.push_back( nh_.subscribe(topic_name, 1, &ToroboJointStateController::sourceCallback, this, ros::TransportHints().reliable().tcpNoDelay(true)) ) ;
}

void ToroboJointStateController::setRate(double rate)
{
    publish_rate_ = rate;
}

void ToroboJointStateController::start()
{
    timer_ = nh_.createTimer(ros::Duration(1.0 / (double)publish_rate_), &ToroboJointStateController::timerCallback, this);
}


/*----------------------------------------------------------------------
 Protected Method Implementations
 ----------------------------------------------------------------------*/
void ToroboJointStateController::sourceCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(int i=0; i<msg->name.size(); i++)
    {
        jointdata_map_[msg->name[i]] = JointData(msg->position[i], msg->velocity[i], msg->effort[i]);
    }
}

void ToroboJointStateController::timerCallback(const ros::TimerEvent& e)
{
    sensor_msgs::JointState::Ptr msg(new sensor_msgs::JointState());
    msg->header.stamp = ros::Time::now();

    for(auto itr = jointdata_map_.begin(); itr != jointdata_map_.end(); ++itr)
    {
        msg->name.push_back( itr->first );
        msg->position.push_back( itr->second.position );
        msg->velocity.push_back( itr->second.velocity );
        msg->effort.push_back( itr->second.effort );
    }

    pub_.publish(msg);
}

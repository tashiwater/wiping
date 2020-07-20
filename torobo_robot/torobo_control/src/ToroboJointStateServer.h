/**
 * @file  ToroboJointStateController.h
 * @brief ToroboJointStateController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_JOINT_STATE_SERVER_H__
#define __TOROBO_JOINT_STATE_SERVER_H__


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include "torobo_msgs/GetJointState.h"
#include "torobo_msgs/GetToroboJointState.h"
#include "torobo_msgs/ToroboJointState.h"


class ToroboJointStateServer
{
public:

    ToroboJointStateServer(ros::NodeHandle& node);
    virtual ~ToroboJointStateServer();

    void setRate(double rate);
    void registerController(std::string controller_name);
    void start();

protected:

    ros::NodeHandle& nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::vector<ros::ServiceServer> services_;
    ros::Timer timer_;
    double publish_rate_ = 20;
    sensor_msgs::JointState::ConstPtr joint_state_; // Prohibit overwriting data pointed by the pointer.
    std::vector<ros::Subscriber> sub_list_;
    std::map<std::string, torobo_msgs::ToroboJointState::ConstPtr> torobojointstate_map_;
    std::map<std::string, ros::Publisher> pub_map_;

    void timerCallback(const ros::TimerEvent& e);
    void sourceCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void sourceToroboCallback(const ros::MessageEvent<torobo_msgs::ToroboJointState const>& event);
    bool jointStateService(torobo_msgs::GetJointState::Request &req, torobo_msgs::GetJointState::Response &res);
    bool toroboJointStateService(torobo_msgs::GetToroboJointState::Request &req, torobo_msgs::GetToroboJointState::Response &res);

};


#endif


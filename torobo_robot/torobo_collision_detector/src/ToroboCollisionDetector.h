/**
 * @file  ToroboCollisionDetector.h
 * @brief Torobo cllision detector class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_COLLISION_DETECTOR_H__
#define __TOROBO_COLLISION_DETECTOR_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "torobo_msgs/CheckCollision.h"
#include <sensor_msgs/JointState.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std;

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboCollisionDetector
{
public:
    ToroboCollisionDetector(ros::NodeHandle& node);
    virtual ~ToroboCollisionDetector();

protected:
    void InitializePlanningScene();
    void InitializeJointStates();
    bool CheckCollisionService(torobo_msgs::CheckCollision::Request &req, torobo_msgs::CheckCollision::Response &res);
    void SubsribeJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

    ros::NodeHandle& node_;
    ros::ServiceServer service_;
    ros::Subscriber sub_;

    sensor_msgs::JointState jointState_;
    std::map<std::string, int> jointNameIndexMap_;

    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};

#endif

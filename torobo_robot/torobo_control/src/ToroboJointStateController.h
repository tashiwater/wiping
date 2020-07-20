/**
 * @file  ToroboJointStateController.h
 * @brief ToroboJointStateController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_JOINT_STATE_CONTROLLER_H__
#define __TOROBO_JOINT_STATE_CONTROLLER_H__


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>


class ToroboJointStateController
{
public:

    ToroboJointStateController(ros::NodeHandle& node, std::string name);
    virtual ~ToroboJointStateController();

    void registerController(std::string controller_name);
    void setRate(double rate);
    void start();

protected:

    struct JointData
    {
        double position;
        double velocity;
        double effort;
        JointData() : position(0.0), velocity(0.0), effort(0.0) {}
        JointData(const JointData& org) {position = org.position; velocity = org.velocity; effort = org.effort;}
        JointData(double p, double v, double e) : position(p), velocity(v), effort(e) {}
    };

    ros::NodeHandle& nh_;
    std::string name_;
    ros::Publisher pub_;
    ros::Timer timer_;
    std::map<std::string, JointData> jointdata_map_;
    std::vector<ros::Subscriber> sub_list_;
    double publish_rate_ = 50.0;

    void timerCallback(const ros::TimerEvent& e);
    void sourceCallback(const sensor_msgs::JointState::ConstPtr& msg);

};


#endif


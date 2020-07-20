/**
 * @file  JointTrajectoryControllerStatePublisher.h
 * @brief Joint trajectory controller state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __JOINT_TRAJECTORY_CONTROLLER_STATE_PUBLISHER_H__
#define __JOINT_TRAJECTORY_CONTROLLER_STATE_PUBLISHER_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "AbstractPublisher.h"
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class JointTrajectoryControllerStatePublisher : public AbstractPublisher
{
public:
    class Coef
    {
    public:
        Coef(const double pos=1.0,
             const double vel=1.0,
             const double acc=1.0,
             const double eft=1.0)
        {
            position = pos;
            velocity = vel;
            acceleration = acc;
            effort = eft;
        }
        ~Coef(){}

        double position;
        double velocity;
        double acceleration;
        double effort;
    };

    JointTrajectoryControllerStatePublisher(ros::NodeHandle &node,
                                            const std::string ns,
                                            const std::map<std::string, int> jointsNameIdMap,
                                            const JointTrajectoryControllerStatePublisher::Coef coef=Coef(),
                                            const int queueSize=1);
    virtual ~JointTrajectoryControllerStatePublisher();

    void Publish(const RecvPacket& packet, const ros::Time rosTimeStamp) override;

protected:
    void Initialize(const int queueSize);

    JointTrajectoryControllerStatePublisher::Coef m_coef;
    control_msgs::JointTrajectoryControllerState m_jointTrajectoryControllerState;
};

}

#endif

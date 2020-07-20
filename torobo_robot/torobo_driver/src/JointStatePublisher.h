/**
 * @file  JointStatePublisher.h
 * @brief Joint state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __JOINT_STATE_PUBLISHER_H__
#define __JOINT_STATE_PUBLISHER_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "AbstractPublisher.h"
#include <sensor_msgs/JointState.h>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class JointStatePublisher : public AbstractPublisher
{
public:
    class Coef
    {
    public:
        Coef(const double pos=1.0,
             const double vel=1.0,
             const double eft=1.0)
        {
            position = pos;
            velocity = vel;
            effort = eft;
        }
        ~Coef(){}

        double position;
        double velocity;
        double effort;
    };

    JointStatePublisher(ros::NodeHandle &node,
                        const std::string ns,
                        const std::map<std::string, int> jointsNameIdMap,
                        const JointStatePublisher::Coef coef=Coef(),
                        const int queueSize=1);
    virtual ~JointStatePublisher();

    void Publish(const RecvPacket& packet, const ros::Time rosTimeStamp) override;

protected:
    void Initialize(const int queueSize);

    JointStatePublisher::Coef m_coef;
    sensor_msgs::JointState m_jointState;
};

}

#endif

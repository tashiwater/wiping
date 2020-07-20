/**
 * @file  ToroboJointController.h
 * @brief Torobo joint controller class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_JOINT_CONTROLLER_H__
#define __TOROBO_JOINT_CONTROLLER_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include "ToroboControllerBase.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboJointController : public ToroboControllerBase
{
public:
    ToroboJointController(ros::NodeHandle &node,
                          MasterControllerClient* const client,
                          const std::string controllerName,
                          const std::map<std::string, int> jointsNameIdMap,
                          const int allJointsNum);
    virtual ~ToroboJointController();

protected:
    void InitializePublisher();
    void InitializeSubscriber();
    void InitializeServiceServer();
    bool isNearEqualJointPose(const trajectory_msgs::JointTrajectoryPoint& point, const std::vector<int> idVec, const double sumEpsilon) const;

    void SubsribeJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    void SendTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& point, const std::vector<std::string>& idstrVec);
};

}

#endif

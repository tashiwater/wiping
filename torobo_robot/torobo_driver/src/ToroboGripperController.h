/**
 * @file  ToroboGripperController.h
 * @brief Torobo gripper controller class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_GRIPPER_CONTROLLER_H__
#define __TOROBO_GRIPPER_CONTROLLER_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include "ToroboControllerBase.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/GripperCommand.h>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboGripperController : public ToroboControllerBase
{
public:
    ToroboGripperController(ros::NodeHandle &node,
                            MasterControllerClient* const client,
                            const std::string controllerName,
                            const std::map<std::string, int> jointsNameIdMap,
                            const int allJointsNum,
                            const std::string jointType);
    virtual ~ToroboGripperController();

protected:
    void InitializePublisher();
    void InitializeSubscriber();
    void InitializeServiceServer();

    void SubsribeJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    void SubsribeGripperCommandCallback(const control_msgs::GripperCommand::ConstPtr& msg);

    void SendTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint point, const std::vector<std::string> idstrVec);
    void SendGripperMaxEffort(const double gripperMaxEffort);
    void SendGripperPosition(const double position);
};

}

#endif

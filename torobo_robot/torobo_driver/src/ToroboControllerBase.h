/**
 * @file  ToroboControllerBase.h
 * @brief Torobo controller base class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_CONTROLLER_BASE_H__
#define __TOROBO_CONTROLLER_BASE_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "MasterControllerClient/MasterControllerClient.h"
#include "MasterControllerClient/Communication/Packet/RecvPacket.h"
#include "AbstractPublisher.h"
#include "torobo_msgs/ToroboDynamics.h"
#include "torobo_msgs/ToroboCommand.h"
#include "torobo_msgs/ErrorReset.h"
#include "torobo_msgs/BrakeOff.h"
#include "torobo_msgs/BrakeOn.h"
#include "torobo_msgs/ServoOff.h"
#include "torobo_msgs/ServoOn.h"
#include "torobo_msgs/GetServoState.h"
#include "torobo_msgs/SetControlMode.h"
#include "torobo_msgs/SetZeroEffort.h"
#include "torobo_msgs/ClearTrajectory.h"
#include "torobo_msgs/CancelTrajectory.h"
#include "torobo_msgs/SetRobotControllerParameter.h"
#include "torobo_msgs/SetGeneralOutputRegister.h"
#include "torobo_msgs/SetPayloadParam.h"
#include "torobo_msgs/SendCommonCommand.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboControllerBase
{
public:
    ToroboControllerBase(ros::NodeHandle &node,
                         MasterControllerClient* const client,
                         const std::string controllerName,
                         const std::map<std::string, int> jointsNameIdMap,
                         const int allJointsNum,
                         const std::string jointType="revolute");
    virtual ~ToroboControllerBase();

    void Publish(const RecvPacket& packet, const ros::Time rosTimeStamp);
    void PrintParam();
    void SetJointState(const RecvPacket& recvPacket);

    void SubsribeToroboDynamicsCallback(const torobo_msgs::ToroboDynamics::ConstPtr& msg);
    void SubsribeToroboCommandCallback(const torobo_msgs::ToroboCommand::ConstPtr& msg);

    bool BrakeOffService(torobo_msgs::BrakeOff::Request &req, torobo_msgs::BrakeOff::Response &res);
    bool BrakeOnService(torobo_msgs::BrakeOn::Request &req, torobo_msgs::BrakeOn::Response &res);
    bool ServoOffService(torobo_msgs::ServoOff::Request &req, torobo_msgs::ServoOff::Response &res);
    bool ServoOnService(torobo_msgs::ServoOn::Request &req, torobo_msgs::ServoOn::Response &res);
    bool GetServoStateService(torobo_msgs::GetServoState::Request &req, torobo_msgs::GetServoState::Response &res);
    bool ErrorResetService(torobo_msgs::ErrorReset::Request &req, torobo_msgs::ErrorReset::Response &res);
    bool SetControlModeService(torobo_msgs::SetControlMode::Request &req, torobo_msgs::SetControlMode::Response &res);
    bool SetZeroEffortService(torobo_msgs::SetZeroEffort::Request &req, torobo_msgs::SetZeroEffort::Response &res);
    bool ClearTrajectoryService(torobo_msgs::ClearTrajectory::Request &req, torobo_msgs::ClearTrajectory::Response &res);
    bool CancelTrajectoryService(torobo_msgs::CancelTrajectory::Request &req, torobo_msgs::CancelTrajectory::Response &res);
    bool SetRobotControllerParameterService(torobo_msgs::SetRobotControllerParameter::Request &req, torobo_msgs::SetRobotControllerParameter::Response &res);
    bool SetGeneralOutputRegisterService(torobo_msgs::SetGeneralOutputRegister::Request &req, torobo_msgs::SetGeneralOutputRegister::Response &res);
    bool SetPayloadParamService(torobo_msgs::SetPayloadParam::Request &req, torobo_msgs::SetPayloadParam::Response &res);
    bool SendCommonCommandService(torobo_msgs::SendCommonCommand::Request &req, torobo_msgs::SendCommonCommand::Response &res);
protected:
    void InitializeJointState();
    void PushBackPublisher(AbstractPublisher* const pub);
    void PushBackSubscriber(const ros::Subscriber& sub);
    void PushBackServiceServer(const ros::ServiceServer& srv);

    std::string ParseJointNameStringToIdString(const std::string& jointName) const;
    std::string ParseJointNameStringToIdString(const std::vector<std::string>& jointNameVec) const;
    std::vector<std::string> ParseJointNameStringToIdStringVector(const std::string& jointName) const;
    std::vector<std::string> ParseJointNameStringToIdStringVector(const std::vector<std::string>& jointNameVec) const;
    std::vector<uint8_t> ParseJointNameStringToIdVector(const std::string& jointName) const;
    std::vector<uint8_t> ParseJointNameStringToIdVector(const std::vector<std::string>& jointNameVec) const;
    SendPacket NewSendPacket() const;
    bool SendBySendPacketMethod(std::vector<std::string> jointNames, void (SendPacket::*setMethod)(const std::string&));

    MasterControllerClient* m_client;
    int m_allJointsNum;
    std::map<std::string, int> m_jointsNameIdMap;
    std::map<std::string, int> m_jointsNameNumStrIdMap;
    std::string m_controllerName;
    std::string m_jointType;

    sensor_msgs::JointState m_jointState;

    ros::NodeHandle& m_node;

private:
    std::vector<AbstractPublisher*> m_pubs;
    std::vector<ros::Subscriber> m_subs;
    std::vector<ros::ServiceServer> m_srvs;
};

}

#endif

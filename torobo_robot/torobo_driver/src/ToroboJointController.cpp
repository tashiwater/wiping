/**
 * @file  ToroboJointController.cpp
 * @brief Torobo joint controller class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "ToroboJointController.h"
#include "JointStatePublisher.h"
#include "JointTrajectoryControllerStatePublisher.h"
#include "ToroboJointStatePublisher.h"
#include <math.h>

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Static Method Declarations
 ----------------------------------------------------------------------*/
static bool isNearEqual(const double a, const double b, const double epsilon);

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboJointController::ToroboJointController(ros::NodeHandle &node,
                                             MasterControllerClient* const client,
                                             const std::string controllerName,
                                             const std::map<std::string, int> jointsNameIdMap,
                                             const int allJointsNum)
    : ToroboControllerBase(node, client, controllerName, jointsNameIdMap, allJointsNum)
{
    // Initialize pusb/subs/srvs
    InitializePublisher();
    InitializeSubscriber();
    InitializeServiceServer();
}

ToroboJointController::~ToroboJointController()
{
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboJointController::InitializePublisher()
{
    const double deg2rad = M_PI / 180.0;
    PushBackPublisher(new JointStatePublisher(m_node, m_controllerName, m_jointsNameIdMap, JointStatePublisher::Coef(deg2rad, deg2rad, 1.0), 1));
    PushBackPublisher(new JointTrajectoryControllerStatePublisher(m_node, m_controllerName, m_jointsNameIdMap, JointTrajectoryControllerStatePublisher::Coef(deg2rad, deg2rad, deg2rad, 1.0), 1));
    PushBackPublisher(new ToroboJointStatePublisher(m_node, m_controllerName, m_jointsNameIdMap, ToroboJointStatePublisher::Coef(deg2rad, deg2rad, deg2rad, 1.0), 1));
}

void ToroboJointController::InitializeSubscriber()
{
#if 0
    PushBackSubscriber(m_node.subscribe(m_controllerName + "/torobo_dynamics", 1, &ToroboControllerBase::SubsribeToroboDynamicsCallback, static_cast<ToroboControllerBase*>(this)));
    PushBackSubscriber(m_node.subscribe(m_controllerName + "/torobo_command", 1, &ToroboControllerBase::SubsribeToroboCommandCallback, static_cast<ToroboControllerBase*>(this)));
    PushBackSubscriber(m_node.subscribe(m_controllerName + "/command", 10, &ToroboJointController::SubsribeJointTrajectoryCallback, this));
#else
    PushBackSubscriber(m_node.subscribe(m_controllerName + "/torobo_dynamics", 1, &ToroboControllerBase::SubsribeToroboDynamicsCallback, static_cast<ToroboControllerBase*>(this), ros::TransportHints().reliable().tcpNoDelay(true)));
    PushBackSubscriber(m_node.subscribe(m_controllerName + "/torobo_command", 1, &ToroboControllerBase::SubsribeToroboCommandCallback, static_cast<ToroboControllerBase*>(this), ros::TransportHints().reliable().tcpNoDelay(true)));
    PushBackSubscriber(m_node.subscribe(m_controllerName + "/command", 1, &ToroboJointController::SubsribeJointTrajectoryCallback, this, ros::TransportHints().reliable().tcpNoDelay(true)));
#endif
}

void ToroboJointController::InitializeServiceServer()
{
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/brake_off", &ToroboControllerBase::BrakeOffService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/brake_on", &ToroboControllerBase::BrakeOnService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/servo_off", &ToroboControllerBase::ServoOffService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/servo_on", &ToroboControllerBase::ServoOnService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/get_servo_state", &ToroboControllerBase::GetServoStateService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/error_reset", &ToroboControllerBase::ErrorResetService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/set_control_mode", &ToroboControllerBase::SetControlModeService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/set_zero_effort", &ToroboControllerBase::SetZeroEffortService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/clear_trajectory", &ToroboControllerBase::ClearTrajectoryService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/cancel_trajectory", &ToroboControllerBase::CancelTrajectoryService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/set_robot_controller_parameter", &ToroboControllerBase::SetRobotControllerParameterService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/set_general_output_register", &ToroboControllerBase::SetGeneralOutputRegisterService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/set_payload_param", &ToroboControllerBase::SetPayloadParamService, static_cast<ToroboControllerBase*>(this)));
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/send_common_command", &ToroboControllerBase::SendCommonCommandService, static_cast<ToroboControllerBase*>(this)));
}

void ToroboJointController::SubsribeJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    // ROS_INFO("Subscribe JointTrajectory. length:%d", (int)msg->points.size());

    // Clear all via points
    SendBySendPacketMethod(msg->joint_names, &SendPacket::SetTrajectoryViaClear);

    // Make idstrVec
    vector<string> idstrVec;
    vector<int> idVec;
    for(auto itr = msg->joint_names.begin(); itr != msg->joint_names.end(); ++itr)
    {
        const string jointName = *itr;
        if(m_jointsNameIdMap.count(jointName) == 0)
        {
            ROS_ERROR("Commanded points have invalid joint names --> Failed to run trajectory.");
            return;
        }
        int id = m_jointsNameIdMap.at(jointName);
        string idstr = to_string(id);
        idstrVec.push_back(idstr);
        idVec.push_back(id);
    }

    // Send trajectory points
    int sendPointsNum = 0;
    const int s = (int)msg->points.size();
    for(int i = 0; i < s; i++)
    {
        // skip t == 0 point if position near equal current position
        if(i == 0)
        {
            const float t = msg->points[0].time_from_start.toSec();
            if(isNearEqual(0.0, t, 1e-4) && isNearEqualJointPose(msg->points[i], idVec, 0.01))
            {
                ROS_INFO("Commanded t == 0 point is equal current pose --> skipped");
                continue;
            }
        }
        SendTrajectoryPoint(msg->points[i], idstrVec);
        sendPointsNum++;
    }

    if(sendPointsNum == 0)
    {
        ROS_INFO("Commanded trajectory points is invalid! --> Any trajectory points is not send.");
        return;
    }

    // Start trajectory control
    SendBySendPacketMethod(msg->joint_names, &SendPacket::SetTrajectoryControlStart);
}

void ToroboJointController::SendTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& point, const std::vector<std::string>& idstrVec)
{
    const double rad2deg = 180.0 / M_PI;
    SendPacket sendPacket = NewSendPacket();
    bool uniqueSendPacketFlag = false;

    const size_t posSize = point.positions.size();
    const size_t velSize = point.velocities.size();
    const size_t accSize = point.accelerations.size();

    for(size_t i = 0; i < posSize; i++)
    {
        const float t = point.time_from_start.sec + point.time_from_start.nsec * 1e-9;
        const float pos = float(point.positions[i] * rad2deg);
        float vel = 0.0f;
        if(i < velSize)
        {
            vel = float(point.velocities[i] * rad2deg);
        }
#if 1
        sendPacket.SetTrajectoryPVT(idstrVec[i], pos, vel, t);
        // sendPacket.SetTrajectoryPT_ContinuousSpline(idstrVec[i], pos, t);
        // ROS_INFO("Send TrajectoryPoint[P,V,T] = [%f, %f, %f]", pos, vel, t);

        const RecvPacket recvPacket = m_client->GetLastRecvPacket();
        const uint8_t ctrlMode = recvPacket.m_joint[i].ctrlMode;
        if(ctrlMode == 6 || ctrlMode == 7)
        {
            uniqueSendPacketFlag = true;
        }
#else
        float acc = 0.0f;
        if(i < velSize)
        {
            acc = float(point.accelerations[i] * rad2deg);
        }
        sendPacket.SetTrajectoryPVAT(idstrVec[i], pos, vel, acc, t);
        ROS_INFO("Send TrajectoryPoint[P,V,A,T] = [%f, %f, %f, %f]", pos, vel, acc, t);
#endif
    }
#if 1
    if(uniqueSendPacketFlag)
    {
        m_client->InsertUniqueSendMap("pvt", sendPacket);
    }
    else
    {
        m_client->PushSendQueue(sendPacket);
    }
#else
    m_client->PushSendQueue(sendPacket);
#endif
}

bool ToroboJointController::isNearEqualJointPose(const trajectory_msgs::JointTrajectoryPoint& point, const std::vector<int> idVec, const double sumEpsilon) const
{
    const double deg2rad = M_PI / 180.0;
    const int s = (int)m_jointState.name.size();
    double sum = 0.0;
    for(auto itr = idVec.begin(); itr != idVec.end(); ++itr)
    {
        const int id = *itr;
        if(id >= s)
        {
            return false;
        }
        sum += abs(point.positions[id] - m_jointState.position[id] * deg2rad);
    }
    if(sum > sumEpsilon)
    {
        return false;
    }
    return true;
}

/*----------------------------------------------------------------------
 Static Method Definitions
 ----------------------------------------------------------------------*/
static bool isNearEqual(const double a, const double b, const double epsilon)
{
    return fabs(a - b) < epsilon;
}

}

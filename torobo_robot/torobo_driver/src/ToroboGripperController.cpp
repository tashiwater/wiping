/**
 * @file  ToroboGripperController.cpp
 * @brief Torobo joint controller class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "ToroboGripperController.h"
#include "JointStatePublisher.h"
#include "JointTrajectoryControllerStatePublisher.h"
#include "ToroboJointStatePublisher.h"
#include <math.h>

using namespace std;

namespace torobo
{

const double millimeter2meter = 1.0 / 1000.0;
const double meter2millimeter = 1000.0;
const double deg2rad = M_PI / 180.0;
const double rad2deg = 180.0 / M_PI;
double master2ros = 1.0;
double ros2master = 1.0;

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboGripperController::ToroboGripperController(ros::NodeHandle &node,
                                                 MasterControllerClient* const client,
                                                 const std::string controllerName,
                                                 const std::map<std::string, int> jointsNameIdMap,
                                                 const int allJointsNum,
                                                 const std::string jointType)
    : ToroboControllerBase(node, client, controllerName, jointsNameIdMap, allJointsNum, jointType)
{
    // Initialize pusb/subs/srvs
    InitializePublisher();
    InitializeSubscriber();
    InitializeServiceServer();
}

ToroboGripperController::~ToroboGripperController()
{
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboGripperController::InitializePublisher()
{
    if(m_jointType == "revolute")
    {
        master2ros = deg2rad;
        ros2master = rad2deg;
    }
    else if(m_jointType == "prismatic")
    {
        master2ros = millimeter2meter;
        ros2master = meter2millimeter;
    }
    PushBackPublisher(new JointStatePublisher(m_node, m_controllerName, m_jointsNameIdMap, JointStatePublisher::Coef(master2ros, master2ros, 1.0), 1));
    PushBackPublisher(new JointTrajectoryControllerStatePublisher(m_node, m_controllerName, m_jointsNameIdMap, JointTrajectoryControllerStatePublisher::Coef(master2ros, master2ros, master2ros, 1.0), 1));
    PushBackPublisher(new ToroboJointStatePublisher(m_node, m_controllerName, m_jointsNameIdMap, ToroboJointStatePublisher::Coef(master2ros, master2ros, master2ros, 1.0), 1));
}

void ToroboGripperController::InitializeSubscriber()
{
    PushBackSubscriber(m_node.subscribe(m_controllerName + "/torobo_command", 1, &ToroboControllerBase::SubsribeToroboCommandCallback, static_cast<ToroboControllerBase*>(this), ros::TransportHints().reliable().tcpNoDelay(true)));
    PushBackSubscriber(m_node.subscribe(m_controllerName + "/command", 1, &ToroboGripperController::SubsribeGripperCommandCallback, this, ros::TransportHints().reliable().tcpNoDelay(true)));
#if 0
    PushBackSubscriber(m_node.subscribe(m_controllerName + "/command", 10, &ToroboGripperController::SubsribeJointTrajectoryCallback, this));
#endif
}

void ToroboGripperController::InitializeServiceServer()
{
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
    PushBackServiceServer(m_node.advertiseService(m_controllerName + "/send_common_command", &ToroboControllerBase::SendCommonCommandService, static_cast<ToroboControllerBase*>(this)));
}

void ToroboGripperController::SubsribeJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    ROS_INFO("Subscribe JointTrajectory. length:%d", (int)msg->points.size());

    // Clear all via points
    SendBySendPacketMethod(msg->joint_names, &SendPacket::SetTrajectoryViaClear);

    // Make idstrVec
    vector<string> idstrVec;
    for(auto itr = msg->joint_names.begin(); itr != msg->joint_names.end(); ++itr)
    {
        string jointName = *itr;
        int id = m_jointsNameIdMap.at(jointName);
        string idstr = to_string(id);
        idstrVec.push_back(idstr);
    }

    // Send trajectory points
    for(auto itr = msg->points.begin(); itr != msg->points.end(); ++itr)
    {
        SendTrajectoryPoint(*itr, idstrVec);
    }

    // Start trajectory control
    SendBySendPacketMethod(msg->joint_names, &SendPacket::SetTrajectoryControlStart);
}

void ToroboGripperController::SubsribeGripperCommandCallback(const control_msgs::GripperCommand::ConstPtr& msg)
{
    ROS_INFO("Subscribe GripperCommand. position:%f, max_effort:%f", msg->position, msg->max_effort);
    SendGripperMaxEffort(msg->max_effort);
    SendGripperPosition(msg->position);
}

void ToroboGripperController::SendTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint point, const std::vector<std::string> idstrVec)
{
    SendPacket sendPacket = NewSendPacket();
    for(int i = 0; i < point.positions.size(); i++)
    {
        const float t = point.time_from_start.sec + point.time_from_start.nsec * 1e-9;
        const float pos = float(point.positions[i] * ros2master);
        const float vel = float(point.velocities[i] * ros2master);
        const float acc = float(point.accelerations[i] * ros2master);
        sendPacket.SetTrajectoryPVT(idstrVec[i], pos, vel, t);
        ROS_INFO("Send TrajectoryPoint[P,V,T] = [%f, %f, %f]", pos, vel, t);
    }
    m_client->PushSendQueue(sendPacket);
}

void ToroboGripperController::SendGripperMaxEffort(const double gripperMaxEffort)
{
    SendPacket sendPacket = NewSendPacket();
    sendPacket.SetGripperMaxEffort((float)gripperMaxEffort);
    m_client->PushSendQueue(sendPacket);
}

void ToroboGripperController::SendGripperPosition(const double position)
{
    SendPacket sendPacket = NewSendPacket();
    string idstr = to_string(m_jointsNameIdMap.begin()->second);
    const float pos = (float)(position * ros2master);
    sendPacket.SetRefPosition(idstr, pos);
    m_client->PushSendQueue(sendPacket);
}

}

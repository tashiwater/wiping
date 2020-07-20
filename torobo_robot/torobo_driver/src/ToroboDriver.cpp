/**
 * @file  ToroboDriver.cpp
 * @brief Torobo driver class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "ToroboDriver.h"
#include "ToroboJointController.h"
#include "ToroboGripperController.h"
#include "MasterControllerClient/MasterControllerClientMock.h"

// #define MEASURE_TIME
#ifdef MEASURE_TIME
#include <chrono>
#include <iostream>
static std::chrono::time_point<std::chrono::system_clock> t0;
#endif

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Private Functions
 ----------------------------------------------------------------------*/
std::string CastXmlRpcValueAsString(XmlRpc::XmlRpcValue value);
map<std::string, string> ParseXmlRpcStructToMap(XmlRpc::XmlRpcValue& value);

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboDriver::ToroboDriver(ros::NodeHandle &node, std::string configName)
    : m_node(node)
{
    m_init = false;
    m_isMock = false;
    m_client = NULL;

    if(!GetParamsFromRosParam(configName, m_param))
    {
        return;
    }
    m_node.param<double>("timeoutSec", m_timeoutSec, 0.1);

    // Count all joints num
    m_param.allJointsNum = 0;
    for(auto itr = m_param.controller.begin(); itr != m_param.controller.end(); ++itr)
    {
        m_param.allJointsNum += (int)itr->second.jointsMap.size();
    }
    PrintParam();
}

ToroboDriver::~ToroboDriver()
{
    for(ToroboControllerBase* controller: m_controllers)
    {
        delete controller;
    }

    if(m_client != NULL)
    {
        delete m_client;
        m_client = NULL;
    }
}

bool ToroboDriver::Initialize()
{
    if(m_init)
    {
        return true;
    }

    // Create MasterControllerClient
    bool init = false;
    if(m_param.interface == "Ethernet")
    {
        init = ConnectEthernet(m_param.ip, m_param.port);
    }
    else if(m_param.interface == "USB")
    {
        init = ConnectSerialPort(m_param.com, m_param.baudrate);
    }
    else if(m_param.interface == "mock")    // for debug/test
    {
        m_isMock = true;
        init = ConnectMock();
    }
    else
    {
        ROS_ERROR("interface[%s] is invalid", m_param.interface.c_str());
    }

    if(!init)
    {
        m_init = false;
        ROS_ERROR("fail to connect");
        return false;
    }
    m_init = true;
    ROS_INFO("succeeded to connect");

    // Create controllers
    for(auto itr = m_param.controller.begin(); itr != m_param.controller.end(); ++itr)
    {
        string controllerName = itr->first;

        ToroboControllerBase* controller = NULL;
        if(controllerName.find("gripper") != string::npos)
        {
            controller = new ToroboGripperController(m_node, m_client, controllerName, itr->second.jointsMap, m_param.allJointsNum, itr->second.jointType);
        }
        else
        {
            controller = new ToroboJointController(m_node, m_client, controllerName, itr->second.jointsMap, m_param.allJointsNum);
        }
        if(controller != NULL)
        {
            m_controllers.push_back(controller);
        }
    }
    m_lastRecvTime = ros::Time::now();
    return true;
}

bool ToroboDriver::IsInit()
{
    return m_init;
}

void ToroboDriver::PrintParam()
{
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "JointsNum : " << m_param.allJointsNum << std::endl;
    std::cout << "Interface : " << m_param.interface <<  std::endl;
    std::cout << "IP        : " << m_param.ip <<  std::endl;
    std::cout << "Port      : " << m_param.port <<  std::endl;
    std::cout << "COM Port  : " << m_param.com <<  std::endl;
    std::cout << "Baudrate  : " << m_param.baudrate << std::endl;
    std::cout << "Controller:" <<  std::endl;
    for(auto itr = m_param.controller.begin(); itr != m_param.controller.end(); ++itr)
    {
        std::cout << "  Name: " <<  itr->first << std::endl;
        for(auto jitr = itr->second.jointsMap.begin(); jitr != itr->second.jointsMap.end(); ++jitr)
        {
            std::cout << "    " << jitr->first << ", " << jitr->second << std::endl;
        }
        std::cout << "  Joint Type: " <<  itr->second.jointType << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl;
}

void ToroboDriver::Publish()
{
    // Update client current status by receiving packets from master controller
    int32_t recvNum = m_client->ReceiveStatus();
    //ROS_ERROR_STREAM("ToroboDriver[" << this << "] : publish thread id = " << std::this_thread::get_id());

    // Get current status vector
    std::vector<RecvPacket> curStateVec = m_client->GetCurStatePacket();

    // Get current time
    ros::Time t = ros::Time::now();

    // Judge timeout & reconnect
    if(recvNum < 1)
    {
        ros::Duration diff = t - m_lastRecvTime;
        if(diff.toSec() > m_timeoutSec)
        {
            ROS_ERROR("Connection timeout. Try to re-connect...");
            bool ret = m_client->ReConnect();
            if(!ret)
            {
                ROS_ERROR("Re-connect failed.");
                m_lastRecvTime = t; // update last recv time for next re-connect after re-timeout
                return;
            }
            ROS_INFO("Succeeded to Re-connect.");
        }
        return;
    }
    m_lastRecvTime = t;

    // Publish status as a ROS message
#if 1
    // Publish latest state
    std::vector<RecvPacket>::iterator itr = curStateVec.end();
    --itr;
    if(m_isMock)
    {
        (*itr).m_preData.timeStamp = t.toNSec() / 1000;
    }
    for(auto citr = m_controllers.begin(); citr != m_controllers.end(); ++citr)
    {
        (*citr)->SetJointState(*itr);
        (*citr)->Publish(*itr, t);
    }
#else
    // Publish received all state
    for(std::vector<RecvPacket>::iterator itr = curStateVec.begin();
        itr != curStateVec.end(); ++itr)
    {
        if(m_isMock)
        {
            (*itr).m_preData.timeStamp = t.toNSec() / 1000;
        }
        for(auto citr = m_controllers.begin(); citr != m_controllers.end(); ++citr)
        {
            (*citr)->Publish(*itr, t);
        }
    }
#endif
}

void ToroboDriver::SendCommandToMaster()
{
    m_client->SendPacketInBuffer();
#ifdef MEASURE_TIME
    auto now = chrono::system_clock::now();
    auto usec = chrono::duration_cast<chrono::microseconds>(now - t0).count();
    t0 = now;
    std::cout << "[torobo_driver]Time: " << usec << std::endl;
#endif
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
bool ToroboDriver::GetParamsFromRosParam(std::string configName, torobo::ToroboDriverParam& param)
{
    if(!m_node.hasParam(configName))
    {
        ROS_ERROR("rosparam name: [%s] is not found.", configName.c_str());
        return false;
    }
    XmlRpc::XmlRpcValue params;
    m_node.getParam(configName, params);
    for(int i = 0; i < params.size(); i++)
    {
        string controllerName = "";
        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator itr = params[i].begin(); itr != params[i].end(); ++itr)
        {
            string name = itr->first;
            XmlRpc::XmlRpcValue value = itr->second;
            if(name == "connection_type")
            {
                param.interface = static_cast<string>(value);
            }
            else if(name == "ethernet")
            {
                map<string, string> m = ParseXmlRpcStructToMap(value);
                param.ip = m["master_controller_ip"];
                param.port = stoi(m["master_controller_port"]);
            }
            else if(name == "serial_port")
            {
                map<string, string> m = ParseXmlRpcStructToMap(value);
                param.com = m["master_controller_device_name"];
                param.baudrate = stoi(m["master_controller_baudrate"]);
            }
            else if(name == "controller_name")
            {
                controllerName = CastXmlRpcValueAsString(value);
                if(param.controller.count(controllerName) == 0)
                {
                    param.controller.insert(make_pair(controllerName, ControllerParam()));
                }
            }
            else if(name == "joints")
            {
                map<string, string> m = ParseXmlRpcStructToMap(value);
                for(auto it = m.begin(); it != m.end(); ++it)
                {
                    string jointName = it->first;
                    int jointIdx = stoi(it->second);
                    param.controller[controllerName].jointsMap.insert(make_pair(jointName, jointIdx));
                }
            }
            else if(name == "joint_type")
            {
                string jointType = CastXmlRpcValueAsString(value);
                param.controller[controllerName].jointType = jointType; 
            }
        }
    }
    return true;
}

bool ToroboDriver::ConnectEthernet(std::string ip, int port)
{
    m_client = new MasterControllerClient(m_param.allJointsNum);
    ROS_INFO("Connect to MasterController as TCP client...");
    return m_client->InitAsTcpClient(ip, port);
}

bool ToroboDriver::ConnectSerialPort(std::string comport, int baudrate)
{
    m_client = new MasterControllerClient(m_param.allJointsNum);
    ROS_INFO("Connect to MasterController as SerialPort client...");
    return m_client->InitAsSerialClient(comport, baudrate);
}

bool ToroboDriver::ConnectMock()
{
    m_client = new MasterControllerClientMock(m_param.allJointsNum);
    ROS_INFO("Connect to MasterControllerClientMock");
    return true;
}

/*----------------------------------------------------------------------
 Prrivate Method Definitions
 ----------------------------------------------------------------------*/
std::string CastXmlRpcValueAsString(XmlRpc::XmlRpcValue value)
{
    switch(value.getType())
    {
    case XmlRpc::XmlRpcValue::TypeDouble:
        return to_string(static_cast<double>(value));
    case XmlRpc::XmlRpcValue::TypeInt:
        return to_string(static_cast<int>(value));
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return to_string(static_cast<bool>(value));
    case XmlRpc::XmlRpcValue::TypeString:
        return static_cast<string>(value);
    };
    return "";
}

map<std::string, string> ParseXmlRpcStructToMap(XmlRpc::XmlRpcValue& value)
{
    map<string, string> xmlMap;
    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator itr = value.begin();
        itr != value.end(); ++itr)
    {
        string key = itr->first;
        XmlRpc::XmlRpcValue v = itr->second;
        xmlMap.insert(make_pair(key, CastXmlRpcValueAsString(v)));
    }

    return xmlMap;
}

}

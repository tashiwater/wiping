/**
 * @file  ToroboDriver.h
 * @brief Torobo driver class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_DRIVER_H__
#define __TOROBO_DRIVER_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "MasterControllerClient/MasterControllerClient.h"
#include "ToroboControllerBase.h"

namespace torobo
{

class ControllerParam
{
public:
    ControllerParam(){}
    ~ControllerParam(){}
    std::map<std::string, int> jointsMap;
    std::string jointType;
};

class ToroboDriverParam
{
public:
    ToroboDriverParam()
    {
        controller.clear();
        allJointsNum = 0;
        interface = "";
        port = 0;
        com = "";
        baudrate = 0;
    }
    virtual ~ToroboDriverParam(){}

    std::map<std::string, ControllerParam> controller;
    int allJointsNum;
    std::string interface;
    std::string ip;
    int port;
    std::string com;
    int baudrate;
};

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboDriver
{
public:
    ToroboDriver(ros::NodeHandle &node, std::string configName);
    virtual ~ToroboDriver();

    bool Initialize();
    bool IsInit();

    void PrintParam();
    void Publish();
    void SendCommandToMaster();

protected:
    bool GetParamsFromRosParam(std::string configName, torobo::ToroboDriverParam& param);
    bool ConnectEthernet(std::string ip, int port);
    bool ConnectSerialPort(std::string comport, int baudrate);
    bool ConnectMock();

    bool m_init;
    bool m_isMock;
    ToroboDriverParam m_param;
    MasterControllerClient* m_client;
    std::vector<ToroboControllerBase*> m_controllers;

    double m_timeoutSec;
    ros::Time m_lastRecvTime;

    ros::NodeHandle& m_node;
};

}

#endif

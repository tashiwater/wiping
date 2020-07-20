/**
 * @file  ToroboDynamics.h
 * @brief Torobo Dynamics class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_DYNAMICS_H__
#define __TOROBO_DYNAMICS_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <string>
#include <map>
#include <memory>
#include "ToroboState.h"
#include "ToroboRbdlModel.h"
#include "torobo_msgs/SetPayloadParam.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboDynamics
{
public:
    ToroboDynamics(ros::NodeHandle& node, const std::map<std::string, int>& nameJointsNumMap);
    virtual ~ToroboDynamics();

    void Update(const std::unique_ptr<ToroboState>& toroboState);
    void Publish();

protected:
    void InitializeToroboDynamics(torobo_msgs::ToroboDynamics& dyna, const std::string& name, const int jointsNum) const;
    void SetRefDynamicsEffort(const std::string& name, const std::vector<double>& effort);
    void SetCurDynamicsEffort(const std::string& name, const std::vector<double>& effort);
    void SetInertiaDiagonal(const std::string& name, const std::vector<double>& diagonalElem);
    void SetGravityCompensationEffort(const std::string& name, const std::vector<double>& gravityCompEffort);
    bool SetPayloadParamService(torobo_msgs::SetPayloadParam::Request &req , torobo_msgs::SetPayloadParam::Response &res);

    ToroboRbdlModel* model_;

    ros::NodeHandle& m_node;
    std::map<std::string, ros::Publisher> m_torobo_dynamics_pub;
    std::map<std::string, ros::ServiceServer> m_service;

    std::map<std::string, int> m_nameJointsNumMap;
    std::map<std::string, torobo_msgs::ToroboDynamics> m_dynamics;
private:
};

}

#endif

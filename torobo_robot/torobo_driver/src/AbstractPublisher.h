/**
 * @file  AbstractPublisher.h
 * @brief Abstract publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __ABSTRACT_PUBLISHER_H__
#define __ABSTRACT_PUBLISHER_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "MasterControllerClient/MasterControllerClient.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class AbstractPublisher
{
public:
    AbstractPublisher(ros::NodeHandle &node, const std::string ns, const std::map<std::string, int> jointsNameIdMap)
    : m_node(node), m_ns(ns), m_jointsNameIdMap(jointsNameIdMap) {}
    virtual ~AbstractPublisher() {}
                                         
    virtual void Publish(const RecvPacket& packet, const ros::Time rosTimeStamp) = 0;

protected:
    ros::NodeHandle& m_node;
    std::string m_ns;
    std::map<std::string, int> m_jointsNameIdMap;
    ros::Publisher m_pub;
};

}

#endif

/**
 * @file  TeachingPointManager.h
 * @brief Teaching point manager class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TEACHING_POINT_MANAGER_H__
#define __TEACHING_POINT_MANAGER_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <vector>
#include "torobo_msgs/DeleteTeachingPoint.h"
#include "torobo_msgs/GetTeachingPoint.h"
#include "torobo_msgs/GetTeachingPointNames.h"
#include "torobo_msgs/RecordTeachingPoint.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

class TeachingPointManager
{
public:
    TeachingPointManager();
    virtual ~TeachingPointManager();

protected:
    bool DeleteTeachingPointService(torobo_msgs::DeleteTeachingPoint::Request &req, torobo_msgs::DeleteTeachingPoint::Response &res);
    bool GetTeachingPointService(torobo_msgs::GetTeachingPoint::Request &req, torobo_msgs::GetTeachingPoint::Response &res);
    bool GetTeachingPointNamesService(torobo_msgs::GetTeachingPointNames::Request &req, torobo_msgs::GetTeachingPointNames::Response &res);
    bool RecordTeachingPointService(torobo_msgs::RecordTeachingPoint::Request &req, torobo_msgs::RecordTeachingPoint::Response &res);

private:
    std::vector<std::string> GetTeachingPointNames();
    
    ros::NodeHandle m_node;
    std::vector<ros::ServiceServer> m_serviceVec;
};

}

#endif

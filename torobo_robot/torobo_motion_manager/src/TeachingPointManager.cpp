/**
 * @file  TeachingPointManager.cpp
 * @brief Teaching point manager class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "TeachingPointManager.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <set>
#include <math.h>

using namespace std;

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

TeachingPointManager::TeachingPointManager()
{
	m_serviceVec.push_back(m_node.advertiseService("delete_teaching_point", &TeachingPointManager::DeleteTeachingPointService, this));
	m_serviceVec.push_back(m_node.advertiseService("get_teaching_point", &TeachingPointManager::GetTeachingPointService, this));
	m_serviceVec.push_back(m_node.advertiseService("get_teaching_point_names", &TeachingPointManager::GetTeachingPointNamesService, this));
	m_serviceVec.push_back(m_node.advertiseService("record_teaching_point", &TeachingPointManager::RecordTeachingPointService, this));
}

TeachingPointManager::~TeachingPointManager()
{
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
bool TeachingPointManager::DeleteTeachingPointService(torobo_msgs::DeleteTeachingPoint::Request &req, torobo_msgs::DeleteTeachingPoint::Response &res)
{
    const string paramName = ros::this_node::getNamespace() + "/teaching_points/" + req.teachingPointName;
    const string nameParamName = ros::this_node::getNamespace() + "/teaching_points/names";
    if(!m_node.hasParam(paramName))
    {
        res.success = false;
        return true;
    }
    m_node.deleteParam(paramName);

    // delete name from teaching_point/names
    vector<string> names = GetTeachingPointNames();
    set<string> setnames(names.begin(), names.end());
    setnames.erase(req.teachingPointName);
    vector<string> newnames(setnames.begin(), setnames.end());
    m_node.setParam(nameParamName, newnames);

    res.success = true;
    return true;
}

bool TeachingPointManager::GetTeachingPointService(torobo_msgs::GetTeachingPoint::Request &req, torobo_msgs::GetTeachingPoint::Response &res)
{
    const string paramName = ros::this_node::getNamespace() + "/teaching_points/" + req.teachingPointName;
    if(!m_node.hasParam(paramName))
    {
        res.success = false;
        return true;
    }
    vector<double> positions;
    m_node.getParam(paramName, positions);

    for(int i = 0; i < positions.size(); i++)
    {
        res.point.positions.push_back(positions[i]);
        res.point.velocities.push_back(0.0f);
        res.point.accelerations.push_back(0.0f);
        res.point.effort.push_back(0.0f);
    }
    res.success = true;
    return true;
}

bool TeachingPointManager::GetTeachingPointNamesService(torobo_msgs::GetTeachingPointNames::Request &req, torobo_msgs::GetTeachingPointNames::Response &res)
{
    vector<string> names = GetTeachingPointNames();
    if(names.size() == 0)
    {
        res.success = false;
        return true;
    }
    res.success = true;
    res.teachingPointNames = names;
    return true;
}

bool TeachingPointManager::RecordTeachingPointService(torobo_msgs::RecordTeachingPoint::Request &req, torobo_msgs::RecordTeachingPoint::Response &res)
{
    const string posParamName = ros::this_node::getNamespace() + "/teaching_points/" + req.teachingPointName;
    const string nameParamName = ros::this_node::getNamespace() + "/teaching_points/names";

    vector<double> positions(req.point.positions);
    vector<string> names = GetTeachingPointNames();
    // duplicate cut
    set<string> setnames(names.begin(), names.end());
    setnames.insert(req.teachingPointName);
    vector<string> newnames(setnames.begin(), setnames.end());

    m_node.setParam(posParamName, positions);
    m_node.setParam(nameParamName, newnames);
    res.success = true;
    return true;
}

std::vector<std::string> TeachingPointManager::GetTeachingPointNames()
{
   vector<string> names;
   names.clear();

   const string paramName = ros::this_node::getNamespace() + "/teaching_points/names";
   if(m_node.hasParam(paramName))
   {
       m_node.getParam(paramName, names);
   }
   return names;
}

}
/**
 * @file  ToroboCollisionDetector.h
 * @brief Torobo cllision detector class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "ToroboCollisionDetector.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

using namespace std;

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/
static const std::string ROBOT_DESCRIPTION = "/torobo/robot_description";

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboCollisionDetector::ToroboCollisionDetector(ros::NodeHandle& node) : node_(node)
{
    InitializePlanningScene();
    InitializeJointStates();
    service_ = node_.advertiseService("check_collision", &ToroboCollisionDetector::CheckCollisionService, this);
    sub_ = node_.subscribe("joint_states", 1, &ToroboCollisionDetector::SubsribeJointStatesCallback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    ROS_INFO("standby ready torobo collision detector!");
}

ToroboCollisionDetector::~ToroboCollisionDetector()
{
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboCollisionDetector::InitializePlanningScene()
{
  	psm_ = planning_scene_monitor::PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));
}

void ToroboCollisionDetector::InitializeJointStates()
{
    planning_scene_monitor::LockedPlanningSceneRW lps(psm_);

    if(lps)
    {
        robot_state::RobotState& state = lps->getCurrentStateNonConst();
        vector<string> jointNames = lps->getCurrentState().getVariableNames();
        for(auto itr = jointNames.begin(); itr != jointNames.end(); ++itr)
        {
            jointState_.name.push_back(*itr);
            jointState_.position.push_back(0.0);
            jointState_.velocity.push_back(0.0);
            jointState_.effort.push_back(0.0);
            jointNameIndexMap_.insert(make_pair(*itr, (int)jointNameIndexMap_.size()));
        }
    }
}

bool ToroboCollisionDetector::CheckCollisionService(torobo_msgs::CheckCollision::Request &req, torobo_msgs::CheckCollision::Response &res)
{
    res.isColliding = false;

    planning_scene_monitor::LockedPlanningSceneRW lps(psm_);

    // ROS_INFO("Send Common Command = [%s, %d, %d, %f, %f, %f, %f, ]", idstr.c_str(), wholeOrder, jointOrder, value1, value2, value3, value4);
    if(lps)
    {
        robot_state::RobotState& state = lps->getCurrentStateNonConst();
        sensor_msgs::JointState& reqJointState = req.jointState;
        sensor_msgs::JointState checkJointState = jointState_;

        // update checkJointState by reqJointState
        const size_t size = reqJointState.name.size();
        for(size_t i = 0; i < reqJointState.name.size(); i++)
        {
            if(jointNameIndexMap_.count(reqJointState.name[i]) > 0)
            {
                const int idx = jointNameIndexMap_[reqJointState.name[i]];
                checkJointState.position[idx] = reqJointState.position[i];
#if 0   // don't use
                checkJointState.velocity[idx] = reqJointState.velocity[i];
                checkJointState.effort[idx] = reqJointState.effort[i];
#endif
            }
        }

        // set checkJointState to robot_state
        for(size_t i = 0; i < checkJointState.name.size(); i++)
        {
            state.setJointPositions(checkJointState.name[i], &checkJointState.position[i]);
            // cout << checkJointState.name[i] << ", " << *state.getJointPositions(checkJointState.name[i]) << endl;;
        }

        collision_detection::CollisionRequest creq;
        collision_detection::CollisionResult cres;
        lps->checkSelfCollision(creq, cres);
        
        res.isColliding = cres.collision;
        // ROS_INFO("Collision detect result:%d", cres.collision ? 1 : 0);
    }
    else
    {
        ROS_ERROR("Planning scene not configured");
        return false;
    }

    return true;
}

void ToroboCollisionDetector::SubsribeJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(size_t i = 0; i < msg->name.size(); i++)
    {
        const string& name = msg->name[i];
        if(jointNameIndexMap_.count(name) > 0)
        {
            const int idx = jointNameIndexMap_[name];
            jointState_.position[idx] = msg->position[i];
            jointState_.velocity[idx] = msg->velocity[i];
            jointState_.effort[idx] = msg->effort[i];
        }
    }
}

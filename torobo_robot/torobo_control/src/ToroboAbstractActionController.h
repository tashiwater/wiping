/**
 * @file  ToroboAbstractActionController.h
 * @brief ToroboAbstractActionController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __TOROBO_ABSTRACT_ACTION_CONTROLLER_H__
#define __TOROBO_ABSTRACT_ACTION_CONTROLLER_H__


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/time.h>


/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/

class ToroboAbstractActionController
{
public:
    ToroboAbstractActionController(ros::NodeHandle& node, std::string name, std::string action_name)
     : nh_(node)
    {
        name_ = name;
        action_name_ = action_name;
    }
    virtual ~ToroboAbstractActionController(){};

protected:
    ros::NodeHandle& nh_;
    std::string name_;
    std::string action_name_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};


#endif


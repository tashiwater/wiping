#include <ros/ros.h>
#include "TeachingPointManager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "torobo_motion_manager_node");
    ROS_INFO("Start torobo_motion_manager");

    torobo::TeachingPointManager tpm;
    ros::spin();

    return 0;
}

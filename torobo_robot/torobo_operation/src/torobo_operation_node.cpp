#include <ros/ros.h>
#include "MoveHomePositionAction.h"
#include "MoveTeachingPointAction.h"
#include "MoveTeachingTrajectoryAction.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "torobo_operation_node");
    ROS_INFO("start torobo_operation_node");

    ros::NodeHandle node;
    while(ros::ok())
    {
        try
        {
            torobo::MoveHomePositionActionServer moveHomePositionServer(node, ros::this_node::getNamespace());
            torobo::MoveTeachingPointActionServer moveTeachingPointServer(node, ros::this_node::getNamespace());
            torobo::MoveTeachingTrajectoryActionServer moveTeachingTrajectoryServer(node, ros::this_node::getNamespace());
            ros::spin();
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM(e.what());
        }
    }

    return 0;
}

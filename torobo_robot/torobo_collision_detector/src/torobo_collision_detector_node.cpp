#include <ros/ros.h>
#include "ToroboCollisionDetector.h"
#include "nodelet/nodelet.h"

namespace torobo {

class CollisionDetector: public nodelet::Nodelet
{
private:
    ToroboCollisionDetector *tcd;

public:

    CollisionDetector()
    {
        tcd = NULL;
    }

    ~CollisionDetector()
    {
        if(tcd)
        {
            delete tcd;
            tcd = NULL;
        }
    }

    void onInit()
    {
        ros::NodeHandle& node = getNodeHandle(); // get NodeHandle for NODELET

        NODELET_INFO("Start torobo_collision_detector");
        tcd = new ToroboCollisionDetector(node);
    }

}; // class CollisionDetector

} // namespace torobo


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::CollisionDetector, nodelet::Nodelet);

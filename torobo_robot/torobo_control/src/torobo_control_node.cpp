#include <ros/ros.h>
#include <string>
#include <map>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>
#include "ToroboAbstractActionController.h"
#include "ToroboJointTrajectoryController.h"
#include "ToroboGripperController.h"
#include "ToroboJointStateController.h"
#include "nodelet/nodelet.h"

using namespace std;

namespace torobo {

class ControllerSpawner: public nodelet::Nodelet
{
private:
    ToroboJointStateController *torobo_joint_state_controller_;
    std::vector<ToroboAbstractActionController*> controllers_;

    typedef struct
    {
        // std::string file;
        double monitor_rate;
    }Args_t;

    bool ParseArgs(int argc, char** argv, Args_t& args)
    {
        boost::program_options::options_description opt("option");
        opt.add_options()
            ("help,h", "show help")
            // ("file", boost::program_options::value<std::string>()->default_value(""), "Default yaml file")
            ("monitor_rate", boost::program_options::value<double>()->default_value(20.0), "Default monitor rate")
            ;
        boost::program_options::variables_map vm;
        try
        {
            boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opt), vm);
        }
        catch(const boost::program_options::error_with_option_name& e)
        {
            NODELET_ERROR("%s", e.what());
            return false;
        }
        boost::program_options::notify(vm);
        if (vm.count("help"))
        {
            std::cout << opt << std::endl;
            return false;
        }

        // parse args
        // args.file = vm["file"].as<std::string>();
        args.monitor_rate = vm["monitor_rate"].as<double>();
        return true;
    }

    void waitParam(ros::NodeHandle &node, std::string name, int timeout=50)
    {
        while(!node.hasParam(name) && timeout > 0)
        {
            ros::Duration(0.1).sleep();
            timeout--;
        }
    }

public:

    ControllerSpawner()
    {
        torobo_joint_state_controller_ = NULL;
    }

    ~ControllerSpawner()
    {
        for(int i=0; i<controllers_.size(); i++)
        {
            delete controllers_[i];
        }
        if(torobo_joint_state_controller_)
        {
            delete torobo_joint_state_controller_;
            torobo_joint_state_controller_ = NULL;
        }
    }

    void onInit()
    {
        //ros::init(argc, argv, "torobo_control_node");

        ros::NodeHandle& node = getNodeHandle(); // get NodeHandle for NODELET

        NODELET_INFO("Start torobo_control");

        const char* argv[32];
        std::vector<std::string> myargv = getMyArgv();
        myargv.insert(myargv.begin(), std::string("dummy"));
        int argc = myargv.size();
        for(int i = 0; i < argc; ++i)
        {
            argv[i] = myargv[i].c_str();
        }

        // Parse args
        Args_t args;
        if(!ParseArgs(argc, (char**)argv, args))
        {
            NODELET_ERROR("Fail to parse arguments");
            return;
        }

        std::vector<std::string> controller_list;
        waitParam(node, "controller_list");
        node.getParam("controller_list", controller_list);

        torobo_joint_state_controller_ = new ToroboJointStateController(node, "joint_state_controller");
        bool use_joint_state_controller = false;

        for(auto itr = controller_list.begin(); itr != controller_list.end(); ++itr)
        {
            std::string controller_name = *itr;
            std::string type;

            waitParam(node, controller_name + "/type");
            node.param<std::string>(controller_name + "/type", type, "");

            if(type.find("JointTrajectoryController") != std::string::npos)
            {
                NODELET_INFO_STREAM("[torobo_control_node] load controller: " << controller_name);
                controllers_.push_back(new ToroboJointTrajectoryController(node, controller_name, "follow_joint_trajectory"));
                torobo_joint_state_controller_->registerController(controller_name);
            }

            else if(type.find("GripperActionController") != std::string::npos)
            {
                NODELET_INFO_STREAM("[torobo_control_node] load controller: " << controller_name);
                controllers_.push_back(new ToroboGripperController(node, controller_name, "gripper_cmd"));
                torobo_joint_state_controller_->registerController(controller_name);
            }

            else if(type.find("JointStateController") != std::string::npos)
            {
                NODELET_INFO_STREAM("[torobo_control_node] load controller: " << controller_name);
                use_joint_state_controller = true;
                double publish_rate = 20.0;// = std::stod(attributes["type"]);
                node.param<double>(controller_name + "/publish_rate", publish_rate, 20.0);
                torobo_joint_state_controller_->setRate(publish_rate);
                NODELET_INFO_STREAM("  publish_rate : " << publish_rate);
            }

        }

        // Check if joint_state_controler is to be used.
        if (use_joint_state_controller)
        {
            torobo_joint_state_controller_->start();
        }
        else
        {
            delete torobo_joint_state_controller_;
            torobo_joint_state_controller_ = NULL;
        }

    }
};

} // namespace torobo

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::ControllerSpawner, nodelet::Nodelet);

#include <ros/ros.h>
#include <string>
#include <map>
#include <boost/program_options.hpp>
#include "ToroboJointStateServer.h"
#include "nodelet/nodelet.h"
#include <yaml-cpp/yaml.h>

using namespace std;

namespace torobo {

class JointStateServer: public nodelet::Nodelet
{
private:
    ToroboJointStateServer* server_;

    typedef struct
    {
        // std::string file;
        double publish_rate;
    }Args_t;

    bool ParseArgs(int argc, char** argv, Args_t& args)
    {
        boost::program_options::options_description opt("option");
        opt.add_options()
            ("help,h", "show help")
            // ("file", boost::program_options::value<std::string>()->default_value(""), "Default yaml file")
            ("publish_rate", boost::program_options::value<double>()->default_value(20.0), "Default publlish rate")
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
        args.publish_rate = vm["publish_rate"].as<double>();
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
    JointStateServer()
    {
        server_ = NULL;
    }

    ~JointStateServer()
    {
        if(server_)
        {
            delete server_;
        }
    }

    void onInit()
    {
        //ros::init(argc, argv, "joint_state_server_node");

        ros::NodeHandle& node = getNodeHandle(); // get NodeHandle for NODELET

        NODELET_INFO("Start torobo_joint_state_server");

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

        double publish_rate = args.publish_rate;
        //ros::param::param<double>("~publish_rate", publish_rate, 20.0);
        NODELET_INFO_STREAM("[torobo_joint_state_server] publish_rate : " << publish_rate);

        server_ = new ToroboJointStateServer(node);
        server_->setRate(publish_rate);

        std::vector<std::string> controller_list;
        waitParam(node, "controller_list");
        node.getParam("controller_list", controller_list);

        for(auto itr = controller_list.begin(); itr != controller_list.end(); ++itr)
        {
            std::string controller_name = *itr;

            std::string type;
            waitParam(node, controller_name + "/type");
            node.param<std::string>(controller_name + "/type", type, "");

            if(type.find("JointTrajectoryController") != std::string::npos)
            {
                NODELET_INFO_STREAM("[torobo_joint_state_server] register controller: " << controller_name);
                server_->registerController(controller_name);
            }

            else if(type.find("GripperActionController") != std::string::npos)
            {
                NODELET_INFO_STREAM("[torobo_joint_state_server] register controller: " << controller_name);
                server_->registerController(controller_name);
            }

        }

        server_->start();
    }

}; // JointStateServer class

} // namespace torobo


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::JointStateServer, nodelet::Nodelet);


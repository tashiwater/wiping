#include <ros/ros.h>
#include <memory>
#include <boost/program_options.hpp>
#include "ToroboState.h"
#include "ToroboDynamics.h"
#include "ToroboRbdlModel.h"
#include "nodelet/nodelet.h"

using namespace std;

namespace torobo {

class ToroboDynamicsNode: public nodelet::Nodelet
{
private:
    std::unique_ptr<torobo::ToroboState> toroboState_;
    std::unique_ptr<torobo::ToroboDynamics> toroboDyna_;
    ros::Timer timer_;

    typedef struct
    {
        bool sim;
        std::string model;
    }Args_t;

private:
    /*----------------------------------------------------------------------
    Function Definitions
    ----------------------------------------------------------------------*/
    bool ParseArgs(int argc, char** argv, Args_t& args)
    {
        boost::program_options::options_description opt("option");
        opt.add_options()
            ("help,h", "show help")
            ("sim", boost::program_options::value<bool>()->default_value(false), "Simulation mode [true] / Real control mode [false]")
            ("model", boost::program_options::value<std::string>()->default_value("toroboarm"), "Robot model [toroboarm]/[torobo]")
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
        args.sim = vm["sim"].as<bool>();
        args.model = vm["model"].as<std::string>();
        return true;
    }

    void timerCallback(const ros::TimerEvent& e)
    {
        ros::spinOnce();

        toroboDyna_->Update(toroboState_);
        toroboDyna_->Publish(); 
    }

public:
    ToroboDynamicsNode()
    {
    }

    ~ToroboDynamicsNode()
    {
    }

    void onInit()
    {
        ros::NodeHandle& node = getNodeHandle(); // get NodeHandle for NODELET

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

        NODELET_INFO("Start torobo_dynamics");

        NODELET_INFO("-----------------------------------------");
        NODELET_INFO("sim  : %s", (args.sim ? "True" : "False")); 
        NODELET_INFO("model: %s", args.model.c_str()); 
        NODELET_INFO("-----------------------------------------");

        map<string, int> nameJointsNumMap;
        if(args.model == "torobo")
        {
            nameJointsNumMap.insert(make_pair("left_arm", 6));
            nameJointsNumMap.insert(make_pair("right_arm", 6));
            nameJointsNumMap.insert(make_pair("torso_head", 4));
        }
        else if(args.model == "toroboarm")
        {
            nameJointsNumMap.insert(make_pair("arm", 7));
        }
        else
        {
            NODELET_ERROR("Invalid model name is given.");
            return;
        }

        toroboState_.reset(new torobo::ToroboState(node, nameJointsNumMap, args.sim));
        toroboDyna_.reset(new torobo::ToroboDynamics(node, nameJointsNumMap));

        timer_ = node.createTimer(ros::Duration(0.002), &ToroboDynamicsNode::timerCallback, this);
    }
}; // class ToroboDynamicsNode

} // namespace torobo

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::ToroboDynamicsNode, nodelet::Nodelet);

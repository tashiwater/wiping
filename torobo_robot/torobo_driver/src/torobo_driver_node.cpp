/**
 * @file  torobo_driver_nor.cpp
 * @brief torobo_driver node implementation
 *
 * @par   Copyright � 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <memory>
#include <boost/program_options.hpp>
#include "ToroboDriver.h"
#include "nodelet/nodelet.h"
#include <boost/thread.hpp>

namespace torobo {

class ToroboDriverNode: public nodelet::Nodelet
{
private:
    std::unique_ptr<torobo::ToroboDriver> toroboDriver_;

    typedef struct
    {
        std::string configName;
    }Args_t;

    /*----------------------------------------------------------------------
    Function Definitions
    ----------------------------------------------------------------------*/
    bool ParseArgs(int argc, char** argv, Args_t& args)
    {
        boost::program_options::options_description opt("option");
        opt.add_options()
            ("help,h", "show help")
            ("config,c", boost::program_options::value<std::string>()->default_value(""), "Torobo controller config rosparam name");
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
        args.configName = vm["config"].as<std::string>();
        return true;
    }

    void procThread()
    {
        while(ros::ok())
        {
            if(!toroboDriver_->IsInit())
            {
                if(!toroboDriver_->Initialize())
                {
                    NODELET_ERROR("Fail to init torobo_driver");
                }
            }
            else
            {
                toroboDriver_->Publish();
                toroboDriver_->SendCommandToMaster();
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }

public:
    ToroboDriverNode()
    {
    }

    ~ToroboDriverNode()
    {
    }

    void onInit()
    {
        ros::NodeHandle& node = getNodeHandle(); // get NodeHandle for NODELET
        NODELET_INFO("Start torobo_driver");

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

        std::string controllerConfigName = node.getNamespace() + "/torobo_controller_config/" + args.configName;
        NODELET_INFO_STREAM("controller config name = " << controllerConfigName);

        // Create ToroboDriver
        toroboDriver_.reset(new torobo::ToroboDriver(node, controllerConfigName));

        // Create thread
        boost::thread thr(&ToroboDriverNode::procThread, this);
        thr.detach();
    }
}; // class Driver

} //namespace torobo

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::ToroboDriverNode, nodelet::Nodelet);

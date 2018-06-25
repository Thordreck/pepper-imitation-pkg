#include <chrono>
#include <thread>
#include <pepper_imitation/UserCommand.h>

#include "PepperTabletNode.hpp"

namespace Pepper
{
    TabletNode::TabletNode() :
        session_ { qi::makeSession() }
    {
        std::string host { "localhost" };
        int port { 9559 };

        node_handle_.param("host", host, host);
        node_handle_.param("port", port, port);

        user_command_publisher_  = node_handle_.advertise<pepper_imitation::UserCommand>("pepper_imitation/cmd_user", 1);

        Connect(host, port);
        tablet_service_.connect("onJSEvent", boost::function<void(std::string)>(boost::bind(&TabletNode::OnDialogInput, this, _1)));
        tablet_service_.connect("onTouchDown", boost::function<void(float, float)>(boost::bind(&TabletNode::OnTouchDown, this, _1, _2)));
    }

    //Private
    void TabletNode::Connect(const std::string& _host, int _port)
    {
        try
        {
            session_->connect("tcp://" + _host + ":" + std::to_string(_port)).wait();
            tablet_service_ = session_->service("ALTabletService");
        }
        catch(const std::exception&)
        {
            ROS_FATAL("Error connecting to %s:%d", _host.c_str(), _port);
            ros::shutdown();
        }
    }

    void TabletNode::OnDialogInput(std::string _input_text)
    {
        pepper_imitation::UserCommand user_command_msg;
        user_command_msg.command = pepper_imitation::UserCommand::INPUT_USER_DATA;
        user_command_msg.args = _input_text;
        user_command_publisher_.publish(user_command_msg);
        input_name_ = true;

        tablet_service_.call<void>("showWebview", "http://10.77.3.117:9999/comptine_image_full.png");
    }

    void TabletNode::OnTouchDown(float _x, float _y)
    {
        if(input_name_) 
        {
            pepper_imitation::UserCommand user_command_msg;
            user_command_msg.command = pepper_imitation::UserCommand::START_GAME;
            user_command_publisher_.publish(user_command_msg);
            return;
        }

        tablet_service_.call<void>("showWebview", "http://198.18.0.1/apps/boot-config/preloading_dialog.html");
        using namespace std::literals::chrono_literals;
        std::this_thread::sleep_for(3s);

        tablet_service_.call<void>("executeJS", script_);
    }
}

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
        tablet_service_.call<void>("showInputTextDialog", "Start Imitation Game?", "Start!", "Maybe later", "Input your name here...", 20);
        tablet_service_.connect("onInputText", boost::function<void(int, std::string)>(boost::bind(&TabletNode::OnDialogInput, this, _1, _2)));
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

    void TabletNode::OnDialogInput(int _validation, const std::string& _input_text)
    {
        pepper_imitation::UserCommand user_command_msg;
        user_command_msg.command = pepper_imitation::UserCommand::START_GAME;
        user_command_publisher_.publish(user_command_msg);
    }
}

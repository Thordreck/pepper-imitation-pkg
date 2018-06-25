#include "PepperTTSNode.hpp"

namespace Pepper
{
    TTSNode::TTSNode() :
        session_ { qi::makeSession() }
    {
        std::string host { "localhost" };
        int port { 9559 };

        node_handle_.param("host", host, host);
        node_handle_.param("port", port, port);

        tts_subscriber_ = node_handle_.subscribe("pepper_imitation/cmd_say", 1, &TTSNode::TTSCallback, this);

        Connect(host, port);
    }

    TTSNode::~TTSNode()
    {
        Say("Au revoir!");
    }

    //Private
    void TTSNode::TTSCallback(const std_msgs::String& _text_msg)
    {
        Say(_text_msg.data);
    }

    void TTSNode::Say(const std::string& _text)
    {
        //TODO: does this throw exceptions or some kind of error. Docs are not clear at all.
        tts_service_.async<void>("say", _text);
    }

    void TTSNode::Connect(const std::string& _host, int _port)
    {
        try
        {
            session_->connect("tcp://" + _host + ":" + std::to_string(_port)).wait();
            tts_service_ = session_->service("ALTextToSpeech");
        }
        catch(const std::exception&)
        {
            ROS_FATAL("Error connecting to %s:%d", _host.c_str(), _port);
            ros::shutdown();
        }
    }
}

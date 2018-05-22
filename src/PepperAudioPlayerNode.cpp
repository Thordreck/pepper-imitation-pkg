#include <std_msgs/Float32.h>

#include "PepperAudioPlayerNode.hpp"

namespace Pepper
{
    AudioPlayerNode::AudioPlayerNode() :
        session_ { qi::makeSession() }
    {
        std::string host { "localhost" };
        int port { 9559 };

        node_handle_.param("host", host, host);
        node_handle_.param("port", port, port);

        commands_subscriber_ = node_handle_.subscribe("pepper_imitation/cmd_audio_player", 1, &AudioPlayerNode::CommandsCallback, this);
        progress_publisher_  = node_handle_.advertise<std_msgs::Float32>("pepper_imitation/audio_player_progress", 1);

        Connect(host, port);
    }

    void AudioPlayerNode::Loop()
    {
        PublishProgress();
        loop_rate_.sleep();
    }

    //Private
    void AudioPlayerNode::CommandsCallback(const pepper_imitation::AudioPlayerCommand& _command_msg)
    {
        switch(_command_msg.command)
        {
            case pepper_imitation::AudioPlayerCommand::PLAY:
                Play(_command_msg.file);
                break;
            case pepper_imitation::AudioPlayerCommand::STOP:
                Stop();
                break;
            case pepper_imitation::AudioPlayerCommand::GOTO:
                GoTo(_command_msg.time);
                break;
            default:
                ROS_WARN("Unrecognized audio player command");
                break;
        }
    }

    void AudioPlayerNode::PublishProgress()
    {
        std_msgs::Float32 progress_msg;
        progress_msg.data = GetProgress();
        progress_publisher_.publish(progress_msg);
    }

    void AudioPlayerNode::Play(const std::string& _file)
    {
        file_task_id_ = audio_player_service_.call<int>("loadFile", _file);
        audio_player_service_.call<void>("play", file_task_id_);
    }

    void AudioPlayerNode::Stop()
    {
        audio_player_service_.call<void>("stopAll");
    }

    void AudioPlayerNode::GoTo(uint16_t _seconds)
    {
        audio_player_service_.call<void>("goTo", file_task_id_, _seconds);
    }

    float AudioPlayerNode::GetProgress()
    {
        return audio_player_service_.call<float>("getCurrentPosition", file_task_id_);
    }

    void AudioPlayerNode::Connect(const std::string& _host, int _port)
    {
        try
        {
            session_->connect("tcp://" + _host + ":" + std::to_string(_port)).wait();
            audio_player_service_ = session_->service("ALAudioPlayer");
        }
        catch(const std::exception&)
        {
            ROS_FATAL("Error connecting to %s:%d", _host.c_str(), _port);
            ros::shutdown();
        }
    }
}

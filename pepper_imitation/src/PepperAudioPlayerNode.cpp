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
        node_handle_.param("audio_folder", audio_folder_, audio_folder_);

        commands_subscriber_ = node_handle_.subscribe("pepper_imitation/cmd_audio_player", 1, &AudioPlayerNode::CommandsCallback, this);
        progress_publisher_  = node_handle_.advertise<std_msgs::Float32>("pepper_imitation/audio_player_progress", 1);

        Connect(host, port);
    }

    AudioPlayerNode::~AudioPlayerNode()
    {
        Stop();
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
            case pepper_imitation::AudioPlayerCommand::PAUSE:
                Pause();
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
        progress_msg.data = file_task_id_ == -1 ? 0.0 : GetProgress();
        progress_publisher_.publish(progress_msg);
    }

    void AudioPlayerNode::Play(const std::string& _file)
    {
        Stop();
        file_task_id_ = audio_player_service_.async<int>("loadFile", audio_folder_ + _file).wait();
        SetVolume(0.8);
        audio_player_service_.async<void>("play", file_task_id_);
    }

    void AudioPlayerNode::Pause()
    {
        if(file_task_id_ == -1) { return; }
        audio_player_service_.async<void>("pause", file_task_id_);
    }
    
    void AudioPlayerNode::Stop()
    {
        audio_player_service_.async<void>("stopAll");
    }

    void AudioPlayerNode::SetVolume(float _volume)
    {
        if(file_task_id_ == -1) { return; }
        audio_player_service_.async<void>("setVolume", file_task_id_, _volume);
    }

    void AudioPlayerNode::GoTo(uint16_t _seconds)
    {
        if(file_task_id_ == -1) { return; }
        Pause();
        audio_player_service_.async<void>("goTo", file_task_id_, _seconds);
    }

    float AudioPlayerNode::GetProgress()
    {
        return audio_player_service_.async<float>("getCurrentPosition", file_task_id_);
    }

    void AudioPlayerNode::Connect(const std::string& _host, int _port)
    {
        try
        {
            session_->connect("tcp://" + _host + ":" + std::to_string(_port)).wait();
            audio_player_service_ = session_->service("ALAudioPlayer");
            Stop();
        }
        catch(const std::exception&)
        {
            ROS_FATAL("Error connecting to %s:%d", _host.c_str(), _port);
            ros::shutdown();
        }
    }
}

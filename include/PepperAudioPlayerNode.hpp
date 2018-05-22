#ifndef PEPPER_AUDIO_PLAYER_NODE_H
#define PEPPER_AUDIO_PLAYER_NODE_H

#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pepper_imitation/AudioPlayerCommand.h>

#include <qi/session.hpp>

namespace Pepper
{
    class AudioPlayerNode final
    {
        public:
            AudioPlayerNode();
            ~AudioPlayerNode() = default;

            void Loop();

        private:
            void Connect(const std::string& _host, int _port);
            void CommandsCallback(const pepper_imitation::AudioPlayerCommand& _command_msg);
            void PublishProgress();

            //Player Commands
            void Play(const std::string& _file);
            void Stop();
            void GoTo(uint16_t _seconds);

            float GetProgress();

        private:
            ros::NodeHandle node_handle_;
            ros::Subscriber commands_subscriber_;
            ros::Publisher  progress_publisher_;
            ros::Rate loop_rate_ { 1 };

            qi::SessionPtr session_;
            qi::AnyObject  audio_player_service_;
            int file_task_id_ { -1 };
    };
}

#endif

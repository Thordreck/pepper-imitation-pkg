#ifndef PEPPER_TTS_NODE_H
#define PEPPER_TTS_NODE_H

#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <qi/session.hpp>

namespace Pepper
{
    class TTSNode final
    {
        public:
            TTSNode();
            ~TTSNode();

        private:
            void Connect(const std::string& _host, int _port);
            void TTSCallback(const std_msgs::String& _text_msg);
            void Say(const std::string& _text);

        private:
            ros::NodeHandle node_handle_;
            ros::Subscriber tts_subscriber_;

            qi::SessionPtr session_;
            qi::AnyObject tts_service_;
    };
}

#endif

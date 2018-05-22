#ifndef PEPPER_TABLET_NODE_H
#define PEPPER_TABLET_NODE_H

#include <string>

#include <ros/ros.h>
#include <qi/session.hpp>

namespace Pepper
{
    class TabletNode final
    {
        public:
            TabletNode();
            ~TabletNode() = default;

        private:
            void Connect(const std::string& _host, int _port);

        private:
            void OnDialogInput(int _validation, const std::string& _input_text);

        private:
            ros::NodeHandle node_handle_;
            ros::Publisher  user_command_publisher_;

            qi::SessionPtr session_;
            qi::AnyObject  tablet_service_;
            qi::AnyObject  event_subscriber_;
    };
}

#endif

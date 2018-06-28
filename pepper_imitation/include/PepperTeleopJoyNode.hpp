#ifndef PEPPER_TELEOP_JOY_NODE_H
#define PEPPER_TELEOP_JOY_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <qi/session.hpp>

namespace Pepper
{
    class TeleopJoyNode final
    {
        public:
            TeleopJoyNode();
            ~TeleopJoyNode() = default;

        private:
            void JoyCallback(const sensor_msgs::Joy& _joy_msg);
            void Connect(const std::string& _host, int _port);
            void Setup();

        private:
            ros::NodeHandle node_handle_;
            ros::Subscriber joy_subscriber_;

            qi::SessionPtr session_;
            qi::AnyObject  motion_service_;
            qi::AnyObject  autonomous_service_;
            qi::AnyObject  background_service_;

            double linear_scale_  { 2.0 };
            double angular_scale_ { 2.0 };

            int x_movement_axis_ { 0 };
            int y_movement_axis_ { 1 };
            int rotation_axis_   { 2 };
    };
}

#endif

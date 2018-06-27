#ifndef PEPPER_TELEOP_JOY_NODE_H
#define PEPPER_TELEOP_JOY_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace Pepper
{
    class TeleopJoyNode final
    {
        public:
            TeleopJoyNode();
            ~TeleopJoyNode() = default;

        private:
            void JoyCallback(const sensor_msgs::Joy& _joy_msg);

        private:
            ros::NodeHandle node_handle_;
            ros::Subscriber joy_subscriber_;
            ros::Publisher  cmd_vel_publisher_;

            double linear_scale_  { 2.0 };
            double angular_scale_ { 2.0 };

            int x_movement_axis_ { 0 };
            int y_movement_axis_ { 1 };
            int rotation_axis_   { 2 };
    };
}

#endif

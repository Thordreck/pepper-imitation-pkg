#include "PepperTeleopJoyNode.hpp"

namespace Pepper
{
    TeleopJoyNode::TeleopJoyNode() :
        session_ { qi::makeSession() }
    {
        node_handle_.param("linear_scale",    linear_scale_,    linear_scale_);
        node_handle_.param("angular_scale",   angular_scale_,   angular_scale_);
        node_handle_.param("x_movement_axis", x_movement_axis_, x_movement_axis_);
        node_handle_.param("y_movement_axis", y_movement_axis_, y_movement_axis_);
        node_handle_.param("rotation_axis",   rotation_axis_,   rotation_axis_);

        std::string host { "localhost" };
        int port { 9559 };

        node_handle_.param("host", host, host);
        node_handle_.param("port", port, port);

        joy_subscriber_    = node_handle_.subscribe("joy", 1, &TeleopJoyNode::JoyCallback, this);
        Connect(host, port);
    }

    //Private
    void TeleopJoyNode::JoyCallback(const sensor_msgs::Joy& _joy_msg)
    {
        motion_service_.call<void>("moveToward", _joy_msg.axes[x_movement_axis_],
                                                  _joy_msg.axes[y_movement_axis_],
                                                  _joy_msg.axes[rotation_axis_]);
        motion_service_.call<void>("moveInit");
    }

    void TeleopJoyNode::Connect(const std::string& _host, int _port)
    {
        try
        {
            session_->connect("tcp://" + _host + ":" + std::to_string(_port)).wait();
            motion_service_  = session_->service("ALMotion");
        }
        catch(const std::exception& ex)
        {
            ROS_FATAL("Error connecting to %s:%d: %s", _host.c_str(), _port, ex.what());
            ros::shutdown();
        }
    }
}

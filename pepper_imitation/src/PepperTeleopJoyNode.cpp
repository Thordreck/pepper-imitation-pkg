#include <geometry_msgs/Twist.h>

#include "PepperTeleopJoyNode.hpp"

namespace Pepper
{
    TeleopJoyNode::TeleopJoyNode()
    {
        node_handle_.param("linear_scale",    linear_scale_,    linear_scale_);
        node_handle_.param("angular_scale",   angular_scale_,   angular_scale_);
        node_handle_.param("x_movement_axis", x_movement_axis_, x_movement_axis_);
        node_handle_.param("y_movement_axis", y_movement_axis_, y_movement_axis_);
        node_handle_.param("rotation_axis",   rotation_axis_,   rotation_axis_);

        joy_subscriber_    = node_handle_.subscribe("joy", 1, &TeleopJoyNode::JoyCallback, this);
        cmd_vel_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    //Private
    void TeleopJoyNode::JoyCallback(const sensor_msgs::Joy& _joy_msg)
    {
        geometry_msgs::Twist twist_msg;

        twist_msg.linear.x = linear_scale_ * _joy_msg.axes[x_movement_axis_];
        twist_msg.linear.y = linear_scale_ * _joy_msg.axes[y_movement_axis_];

        twist_msg.angular.z = angular_scale_ * _joy_msg.axes[rotation_axis_];

        cmd_vel_publisher_.publish(twist_msg);
    }
}

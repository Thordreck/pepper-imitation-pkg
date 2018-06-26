#include <signal.h>
#include <ros/xmlrpc_manager.h>

#include "PepperFaceTrackerNode.hpp"

sig_atomic_t volatile request_shutdown_flag { 0 };

void sigint_handler(int _sig)
{
    request_shutdown_flag = 1;
}

void shut_down_callback(const XmlRpc::XmlRpcValue& _params, XmlRpc::XmlRpcValue& _result)
{
    int num_params { 0 };
    if(_params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        num_params = _params.size();
    }

    if(num_params > 1)
    {
        ROS_WARN("Shutdown request received!");
        request_shutdown_flag = 1;
    }

    _result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pepper_face_tracker_node", ros::init_options::NoSigintHandler);
    ros::Rate rate(20);
    signal(SIGINT, sigint_handler);
    Pepper::FaceTrackerNode pepper_node;
    
    while(!request_shutdown_flag)
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


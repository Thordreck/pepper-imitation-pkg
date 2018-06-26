#include "PepperFaceTrackerNode.hpp"

namespace Pepper
{
    FaceTrackerNode::FaceTrackerNode() :
        session_ { qi::makeSession() }
    {
        std::string host { "localhost" };
        int port { 9559 };

        node_handle_.param("host", host, host);
        node_handle_.param("port", port, port);

        Connect(host, port);

        tracking_subscriber_ = node_handle_.subscribe("pepper_imitation/cmd_set_face_tracking", 1, &FaceTrackerNode::FaceTrackerCallback, this);
    }

    FaceTrackerNode::~FaceTrackerNode()
    {
        EnableFaceTracking(false);
    }

    //Private
    void FaceTrackerNode::FaceTrackerCallback(const std_msgs::Bool& _tracking_msg)
    {
        EnableFaceTracking(_tracking_msg.data);
    }

    void FaceTrackerNode::Connect(const std::string& _host, int _port)
    {
        try
        {
            session_->connect("tcp://" + _host + ":" + std::to_string(_port)).wait();
            face_detection_service_ = session_->service("ALFaceDetection");
        }
        catch(const std::exception& ex)
        {
            ROS_FATAL("Error connecting to %s:%d: %s", _host.c_str(), _port, ex.what());
            ros::shutdown();
        }
    }

    void FaceTrackerNode::EnableFaceTracking(bool _enable)
    {
        face_detection_service_.call<void>("setTrackingEnabled", _enable);
    }
}

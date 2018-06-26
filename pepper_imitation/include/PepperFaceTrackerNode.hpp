#ifndef PEPPER_FACE_TRACKER_NODE_H
#define PEPPER_FACE_TRACKER_NODE_H

#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <qi/session.hpp>

namespace Pepper
{
    class FaceTrackerNode final
    {
        public:
            FaceTrackerNode();
            ~FaceTrackerNode();

        private:
            void Connect(const std::string& _host, int _port);
            void FaceTrackerCallback(const std_msgs::Bool& _tracking_msg);

            void EnableFaceTracking(bool _enable);

        private:
            ros::NodeHandle node_handle_;
            ros::Subscriber tracking_subscriber_;

            qi::SessionPtr session_;
            qi::AnyObject face_detection_service_;
    };
}

#endif

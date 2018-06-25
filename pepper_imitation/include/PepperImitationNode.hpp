#ifndef PEPPER_IMITATION_NODE_H
#define PEPPER_IMITATION_NODE_H

#include <string>
#include <map>
#include <array>
#include <functional>
#include <atomic>
#include <future>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pepper_imitation/ImitationPose.h>

#include <qi/session.hpp>

namespace Pepper
{
    class ImitationNode final
    {
        private:
            using MovementSettings = std::pair<std::vector<std::string>, std::vector<float>>;
        public:
            ImitationNode();
            ~ImitationNode();

        private:
            void Connect(const std::string& _host, int _port);
            void Setup();
            void GoHome();

            void PoseCallback(const pepper_imitation::ImitationPose& _imitation_msg);

            void SetPose(const MovementSettings& _movement_data, float _time);
            void CheckPose(std::function<bool(void)> _check_pose, const std::chrono::seconds& _timeout);
            void StopCheckingCurrentPose();
            void SendResult(uint8_t _result);

            bool CheckHandsUpPose();
            bool CheckHandsOnHeadPose();
            bool CheckHandsOnFrontPose();
            bool CheckHandsOnShoulderPose();
            bool CheckCrossedArmsPose();
            bool CheckHandsOnSidePose();
            bool CheckHandsTogetherPose();
            bool CheckHandOnMouthPose();

            //Utils
            tf::StampedTransform GetTransform(const std::string& _origin_frame, const std::string& _end_frame);

        private:
            ros::NodeHandle node_handle_;
            ros::AsyncSpinner async_spinner_;
            ros::Rate loop_rate_ { 60 };
            ros::Subscriber pose_subscriber_;
            ros::Publisher imitation_result_publisher_;

            qi::SessionPtr session_;
            qi::AnyObject movement_service_;
            qi::AnyObject posture_service_;
            qi::AnyObject autonomous_service_;
            qi::AnyObject background_service_;

            tf::TransformListener tf_listener_;

            std::atomic<bool> check_pose_ { false };
            std::future<void> check_pose_task_;

            const std::map<uint8_t, MovementSettings> poses_map_
            {
                { pepper_imitation::ImitationPose::HANDS_UP,           { {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"}, { -1.57, 0.7853, 0.0, -0.0087, 0.0, -1.57, -0.78, 0.0, 0.0087, 0.0 }}},
                { pepper_imitation::ImitationPose::HANDS_ON_HEAD,      { {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"}, { -1.3, 0.5923, -0.2189, -1.2874, -1.6424, -1.3, -0.5923, 0.2189, 1.6424}}},
                { pepper_imitation::ImitationPose::HANDS_ON_FRONT,     { {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"}, { 0.265, 0.0087, -0.2650, -0.5665, -0.8766, 0.2650, -0.0087, 0.2650, 0.5665, 0.8766}}},
                { pepper_imitation::ImitationPose::HANDS_ON_SHOULDERS, { {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"}, { -1.4, 1.14, -0.24, -1.56, -1.82, -1.30, -1.20, 0.35, 1.56, 1.82} }},
                { pepper_imitation::ImitationPose::CROSSED_ARMS,       { {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"}, { 0.541, 0.008, 0.4033, -0.7296, -1.8238, 1.0025, -0.0087, 0.1959, 1.0128, 0.6751} }},
                { pepper_imitation::ImitationPose::HANDS_ON_SIDES,     { {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"}, { 1.2329, 0.8411, 0.0115, -1.4933, -1.8238, 1.4173, -0.8154, 0.1037, 1.4075, 1.5618} }},
                { pepper_imitation::ImitationPose::HANDS_TOGETHER,     { {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"}, { 0.0576, 0.1975, 0.0115, -1.4933, -1.8238, -0.2881, -0.0774, 0.1037, 1.4075, 1.5618} }},
                { pepper_imitation::ImitationPose::HAND_ON_MOUTH,      { {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"}, { 1.3942, 0.2318, 0.2420, -0.2318, -1.8238, -0.3802, -0.2147, 0.311, 1.5620, 0.8162} }},
            };

            const std::map<uint8_t, std::function<bool(void)>> check_poses_functions_
            {
                { pepper_imitation::ImitationPose::HANDS_UP,            [this] { return CheckHandsUpPose(); } },
                { pepper_imitation::ImitationPose::HANDS_ON_HEAD,       [this] { return CheckHandsOnHeadPose(); } },
                { pepper_imitation::ImitationPose::HANDS_ON_FRONT,      [this] { return CheckHandsOnFrontPose(); } },
                { pepper_imitation::ImitationPose::HANDS_ON_SHOULDERS,  [this] { return CheckHandsOnShoulderPose(); } },
                { pepper_imitation::ImitationPose::CROSSED_ARMS,        [this] { return CheckCrossedArmsPose(); } },
                { pepper_imitation::ImitationPose::HANDS_ON_SIDES,      [this] { return CheckHandsOnSidePose(); } },
                { pepper_imitation::ImitationPose::HANDS_TOGETHER,      [this] { return CheckHandsTogetherPose(); } },
                { pepper_imitation::ImitationPose::HAND_ON_MOUTH,       [this] { return CheckHandOnMouthPose(); } },
            };

            std::string skeleton_tracker_base_frame_ { "/camera_link" };
    };
}

#endif

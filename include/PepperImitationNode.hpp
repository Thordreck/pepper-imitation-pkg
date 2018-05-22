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
#include <moveit/move_group_interface/move_group_interface.h>
#include <pepper_imitation/ImitationPose.h>

namespace Pepper
{
    class ImitationNode final
    {
        public:
            using PlannerInterface = moveit::planning_interface::MoveGroupInterface;
            using Plan             = moveit::planning_interface::MoveGroupInterface::Plan;

        public:
            ImitationNode();
            ~ImitationNode();

        private:
            void PoseCallback(const pepper_imitation::ImitationPose& _imitation_msg);

            void SetUpInterface(PlannerInterface& _interface);
            void ResetInterface(PlannerInterface& _interface);
            void PlanTrajectory(PlannerInterface& _interface);
            void ExecuteTrajectory(PlannerInterface& _interface);

            void CheckPose(std::function<bool(void)> _check_pose, const std::chrono::seconds& _timeout);
            void StopCheckingCurrentPose();
            void SendResult(uint8_t _result);

            bool CheckHandsUpPose();
            bool CheckHandsOnHeadPose();

            //Utils
            tf::StampedTransform GetTransform(const std::string& _origin_frame, const std::string& _end_frame);
            tf::Vector3          ToROSAxis(const tf::Vector3& _vector);

        private:
            ros::NodeHandle node_handle_;
            ros::AsyncSpinner async_spinner_;
            ros::Rate loop_rate_ { 60 };
            ros::Subscriber pose_subscriber_;
            ros::Publisher imitation_result_publisher_;

            tf::TransformListener tf_listener_;

            PlannerInterface both_arms_interface_ { "both_arms" };

            std::atomic<bool> check_pose_ { false };
            std::future<void> check_pose_task_;

            const std::map<uint8_t, std::string> pose_name_map_
            {
                { pepper_imitation::ImitationPose::HANDS_UP,      "hands_up" },
                { pepper_imitation::ImitationPose::HANDS_ON_HEAD, "hands_on_head" },
            };

            const std::map<uint8_t, std::function<bool(void)>> check_poses_functions_
            {
                { pepper_imitation::ImitationPose::HANDS_UP,      [this] { return CheckHandsUpPose(); } },
                { pepper_imitation::ImitationPose::HANDS_ON_HEAD, [this] { return CheckHandsOnHeadPose(); } },
            };

            //Motion planner param settings
            double velocity_scaling_factor_         { 0.1 };
            double acceleration_scaling_factor_     { 0.1 };
            double goal_joint_tolerance_            { 0.3 };
            double goal_pos_tolerance_              { 0.3 };
            double goal_orientation_tolerance_      { 0.3 };
            double max_planning_time_               { 5.0 };
            int    max_planning_attempts_           { 10 };
            std::string motion_planner_             { "RRTConnectionConfigDefault" };
            const std::string pose_reference_frame_ { "/torso" };

            std::string skeleton_tracker_base_frame_ { "/camera_link" };
    };
}

#endif

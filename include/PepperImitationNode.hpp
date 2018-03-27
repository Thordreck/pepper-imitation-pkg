#ifndef PEPPER_IMITATION_NODE_H
#define PEPPER_IMITATION_NODE_H

#include <string>
#include <map>
#include <array>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace Pepper
{
    class ImitationNode final
    {
        public:
            using PlannerInterface = moveit::planning_interface::MoveGroupInterface;
            using Plan             = moveit::planning_interface::MoveGroupInterface::Plan;
            using JointsMap        = std::map<std::string, double>;
            using RPY              = std::array<double, 3>;

        public:
            ImitationNode();
            ~ImitationNode();

            void Loop();

        private:
            void MoveArms();
            void PlanTrajectory(PlannerInterface& _interface);
            void ExecuteTrajectory(PlannerInterface& _interface);
            void SetUpInterface(PlannerInterface& _interface, JointsMap& _joints);
            void ResetInterface(PlannerInterface& _interface);
            void SetJointsToCurrentState(PlannerInterface& _interface, JointsMap& _joints);
            RPY  GetFrameRPY(const std::string& _origin_frame, const std::string& _end_frame);

        private:
            ros::NodeHandle node_handle_;
            ros::AsyncSpinner async_spinner_;

            tf::TransformListener tf_listener_;

            PlannerInterface arms_interface_ { "both_arms" };
            PlannerInterface head_interface_ { "head" };

            JointsMap arms_joints_;
            JointsMap head_joints_;

            std::string skeleton_tracker_base_frame_ { "/camera_link" };

            //Motion planner param settings
            double velocity_scaling_factor_     { 0.1 };
            double acceleration_scaling_factor_ { 0.1 };
            double goal_joint_tolerance_        { 0.3 };
            double max_planning_time_           { 5.0 };
            int    max_planning_attempts_       { 10 };
            std::string motion_planner_         { "RRTConnectionConfigDefault" };
    };
}

#endif

#include <thread>
#include <chrono>
#include <algorithm>

#include <tf/transform_datatypes.h>

#include "PepperImitationNode.hpp"

namespace Pepper
{
    ImitationNode::ImitationNode() :
        async_spinner_ { 1 }
    {
        node_handle_.param("skeleton_tracker_base_frame", skeleton_tracker_base_frame_, skeleton_tracker_base_frame_);
        node_handle_.param("velocity_scaling_factor",     velocity_scaling_factor_,     velocity_scaling_factor_);
        node_handle_.param("acceleration_scaling_factor", acceleration_scaling_factor_, acceleration_scaling_factor_);
        node_handle_.param("goal_joint_tolerance",        goal_joint_tolerance_,        goal_joint_tolerance_);
        node_handle_.param("goal_position_tolerance",     goal_pos_tolerance_,          goal_pos_tolerance_);
        node_handle_.param("goal_orientation_tolerance",  goal_orientation_tolerance_,  goal_orientation_tolerance_);
        node_handle_.param("max_planning_time_",          max_planning_time_,           max_planning_time_);
        node_handle_.param("max_planning_attempts_",      max_planning_attempts_,       max_planning_attempts_);
        node_handle_.param("motion_planner_",             motion_planner_,              motion_planner_);

        async_spinner_.start();

        //SetUpInterface(arms_interface_, arms_joints_);
        //SetUpInterface(head_interface_, head_joints_);
    }

    ImitationNode::~ImitationNode()
    {
        /*
        ResetInterface(arms_interface_);
        ResetInterface(head_interface_);
        */
    }

    void ImitationNode::Loop()
    {
        /*
        MoveArmsJoints();
        //MoveArmsTrajectory();
        using namespace std::literals::chrono_literals;
        std::this_thread::sleep_for(1s);
        */
        try
        {
            //const auto& shoulder_to_torso_vector = GetTransform("/left_shoulder_1", "/torso_1").getOrigin();
            const auto& shoulder_to_elbow_original_vector = GetTransform("/left_shoulder_1", "/left_elbow_1").getOrigin();
            const auto& origin_to_shoulder_vector = ToROSAxis(GetTransform(skeleton_tracker_base_frame_, "left_shoulder_1").getOrigin());
            const auto& origin_to_elbow_vector = ToROSAxis(GetTransform(skeleton_tracker_base_frame_, "left_elbow_1").getOrigin());
            const auto& shoulder_to_elbow_vector = origin_to_elbow_vector - origin_to_shoulder_vector;
            //ROS_INFO("Shoulder to torso original (%f, %f, %f)", shoulder_to_torso_vector.x(), shoulder_to_torso_vector.y(), shoulder_to_torso_vector.z());
            ROS_INFO("Origin to shoulder original (%f, %f, %f)", origin_to_shoulder_vector.x(), origin_to_shoulder_vector.y(), origin_to_shoulder_vector.z());
            ROS_INFO("Origin to elbow original (%f, %f, %f)", origin_to_elbow_vector.x(), origin_to_elbow_vector.y(), origin_to_elbow_vector.z());
            ROS_INFO("Shoulder to elbow computed (%f, %f, %f)", shoulder_to_elbow_vector.x(), shoulder_to_elbow_vector.y(), shoulder_to_elbow_vector.z());
            ROS_INFO("Shoulder to elbow original (%f, %f, %f)", shoulder_to_elbow_original_vector.x(), shoulder_to_elbow_original_vector.y(), shoulder_to_elbow_original_vector.z());
            //ROS_ERROR("Left elbow roll angle default %f", (shoulder_to_torso_vector.angle(shoulder_to_elbow_vector) * 180.0) / M_PI);

            /*
            tf::Vector3 shoulder_to_torso_ros { shoulder_to_torso_vector.z(), shoulder_to_torso_vector.y(), 0 };
            tf::Vector3 shoulder_to_elbow_ros { shoulder_to_elbow_vector.z(), shoulder_to_elbow_vector.y(), 0 };
            const auto& shoulder_to_torso_ros_normalized = shoulder_to_torso_ros.normalized();
            const auto& shoulder_to_elbow_ros_normalized = shoulder_to_elbow_ros.normalized();
            ROS_INFO("Shoulder to torso ROS (%f, %f, %f)", shoulder_to_torso_ros.x(), shoulder_to_torso_ros.y(), shoulder_to_torso_ros.z());
            ROS_INFO("Shoulder to elbow ROS (%f, %f, %f)", shoulder_to_elbow_ros.x(), shoulder_to_elbow_ros.y(), shoulder_to_elbow_ros.z());
            ROS_INFO("Shoulder to torso ROS normalized (%f, %f, %f)", shoulder_to_torso_ros_normalized.x(), shoulder_to_torso_ros_normalized.y(), shoulder_to_torso_ros_normalized.z());
            ROS_INFO("Shoulder to elbow ROS normalized (%f, %f, %f)", shoulder_to_elbow_ros_normalized.x(), shoulder_to_elbow_ros_normalized.y(), shoulder_to_elbow_ros_normalized.z());
            */
            ROS_INFO("----------------------");
            //const auto& cross_product = shoulder_to_torso_normalized.cross(shoulder_to_elbow_normalized).z();
            //const auto& dot_product = shoulder_to_torso_normalized.dot(shoulder_to_elbow_normalized);
            //ROS_INFO("Left elbow roll angle other %f", (std::atan2(cross_product, dot_product) * 180.0) / M_PI);
            //ROS_INFO("Left elbow roll angle TF %f", (shoulder_to_torso.angle(shoulder_to_elbow) * 180.0) / M_PI);
        }
        catch(const tf::TransformException&) {}
        loop_rate_.sleep();
    }

    /*
    //Private
    void ImitationNode::MoveArmsJoints()
    {
        try
        {
            const auto& left_shoulder_rpy  = GetFrameRPY("/left_shoulder_1", "/neck_1");
            const auto& left_elbow_rpy     = GetFrameRPY("/left_elbow_1", "/left_hand_1");

            ROS_INFO("Left shoulder RPY (%f,%f,%f)", left_shoulder_rpy[0], left_shoulder_rpy[1], left_shoulder_rpy[2]);
            ROS_INFO("Left elbow RPY (%f,%f,%f)", left_elbow_rpy[0], left_elbow_rpy[1], left_elbow_rpy[2]);

            arms_joints_["LShoulderRoll"]  = -1.0 * left_shoulder_rpy[2];
            arms_joints_["LShoulderPitch"] = left_shoulder_rpy[0];
            arms_joints_["LElbowRoll"]     = -1.0 * left_elbow_rpy[2];

            const auto& right_shoulder_rpy = GetFrameRPY("/right_shoulder_1", "/neck_1");
            const auto& right_elbow_rpy    = GetFrameRPY("/right_elbow_1", "/right_hand_1");

            ROS_INFO("Right shoulder RPY (%f,%f,%f)", right_shoulder_rpy[0], right_shoulder_rpy[1], right_shoulder_rpy[2]);
            ROS_INFO("Right elbow RPY (%f,%f,%f)", right_elbow_rpy[0], right_elbow_rpy[1], right_elbow_rpy[2]);

            arms_joints_["RShoulderRoll"]  = -1.0 * right_shoulder_rpy[2];
            arms_joints_["RShoulderPitch"] = right_shoulder_rpy[0];
            arms_joints_["RElbowRoll"]     = -1.0 * right_elbow_rpy[2];
        }
        catch(const tf::TransformException& ex)
        {
            ROS_ERROR_THROTTLE(5, "Error getting skeleton data: %s", ex.what());
            return;
        }

        arms_interface_.setStartStateToCurrentState();
        arms_interface_.setJointValueTarget(arms_joints_);

        PlanTrajectory(arms_interface_);
        ExecuteTrajectory(arms_interface_);
    }

    void ImitationNode::MoveArmsTrajectory()
    {
        try
        {
            geometry_msgs::Pose left_hand_pose, goal_pose, torso_pose;
            tf::poseTFToMsg(GetTransform("/torso_1", "/left_hand_1"), left_hand_pose);
            tf::poseTFToMsg(GetTransform("/odom", "/torso"), torso_pose);
            auto current_pose = arms_interface_.getCurrentPose();

            goal_pose.orientation = current_pose.pose.orientation;
            goal_pose.position.x = torso_pose.position.x;
            goal_pose.position.y = -1.0 * left_hand_pose.position.x + torso_pose.position.y;
            goal_pose.position.z = left_hand_pose.position.y + torso_pose.position.z;
            
            
            arms_interface_.setPositionTarget(goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
            ROS_INFO("Current pose [(%f, %f, %f),(%f, %f, %f)]", current_pose.pose.position.x,
                                                                 current_pose.pose.position.y,
                                                                 current_pose.pose.position.z,
                                                                 current_pose.pose.orientation.x,
                                                                 current_pose.pose.orientation.y,
                                                                 current_pose.pose.orientation.z);
            ROS_INFO("Target pose [(%f, %f, %f),(%f, %f, %f)]",  goal_pose.position.x,
                                                                 goal_pose.position.y,
                                                                 goal_pose.position.z,
                                                                 goal_pose.orientation.x,
                                                                 goal_pose.orientation.y,
                                                                 goal_pose.orientation.z);
        }
        catch(const tf::TransformException& ex)
        {
            ROS_ERROR_THROTTLE(5, "Error getting skeleton data: %s", ex.what());
            return;
        }

        PlanTrajectory(arms_interface_);
        ExecuteTrajectory(arms_interface_);
    }

    void ImitationNode::PlanTrajectory(PlannerInterface& _interface)
    {
        Plan movement_plan;
        const auto& result = _interface.plan(movement_plan);
        if(result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_WARN_THROTTLE(5, "Error planning trajectory on %s", _interface.getName().c_str());
        }
    }

    void ImitationNode::ExecuteTrajectory(PlannerInterface& _interface)
    {
        const auto& result = _interface.asyncMove();
        if(result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_WARN_THROTTLE(5, "Error executing trajectory on %s", _interface.getName().c_str());
        }
    }

    void ImitationNode::SetJointsToCurrentState(PlannerInterface& _interface, JointsMap& _joints)
    {
        const auto current_state = _interface.getCurrentState();
        std::vector<double> current_joint_values;
        current_state->copyJointGroupPositions(current_state->getJointModelGroup(_interface.getName()),
                                               current_joint_values);
        auto joint_it = current_joint_values.cbegin();
        for(auto& joint_entry : _joints) { joint_entry.second = *joint_it++; }
    }

    ImitationNode::RPY ImitationNode::GetFrameRPY(const std::string& _origin_frame, const std::string& _end_frame)
    {
        RPY transform_rpy {};
        const auto& transform = GetTransform(_origin_frame, _end_frame);
        static_cast<tf::Matrix3x3>(transform.getRotation()).getRPY(transform_rpy[0], transform_rpy[1], transform_rpy[2]);

        return transform_rpy;
    }

    void ImitationNode::SetUpInterface(PlannerInterface& _interface, JointsMap& _joints)
    {
        _interface.startStateMonitor();

        _interface.setPlannerId(motion_planner_);
        _interface.setMaxVelocityScalingFactor(velocity_scaling_factor_);
        _interface.setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
        _interface.setGoalJointTolerance(goal_joint_tolerance_);
        _interface.setGoalOrientationTolerance(goal_orientation_tolerance_);
        _interface.setGoalPositionTolerance(goal_pos_tolerance_);
        _interface.setPlanningTime(max_planning_time_);
        _interface.setNumPlanningAttempts(max_planning_attempts_);
        _interface.setPoseReferenceFrame(pose_reference_frame_);

        _interface.rememberJointValues("initial_pose");
    }

    void ImitationNode::ResetInterface(PlannerInterface& _interface)
    {
        _interface.setNamedTarget("initial_pose");
        PlanTrajectory(_interface);
        ExecuteTrajectory(_interface);
    }
    */

    tf::StampedTransform ImitationNode::GetTransform(const std::string& _origin_frame, const std::string& _end_frame)
    {
        tf::StampedTransform transform;
        tf_listener_.lookupTransform(_origin_frame, _end_frame, ros::Time(0), transform);

        return transform;
    }

    tf::Vector3 ImitationNode::ToROSAxis(const tf::Vector3& _vector)
    {
        return { -1.0 * _vector.z(), _vector.x(), _vector.y() };
    }
}

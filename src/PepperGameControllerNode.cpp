#include <tf/transform_datatypes.h>
#include <pepper_imitation/ImitationResult.h>

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

        pose_subscriber_            = node_handle_.subscribe("pepper_imitation/cmd_set_pose", 1, &ImitationNode::PoseCallback, this);
        imitation_result_publisher_ = node_handle_.advertise<pepper_imitation::ImitationResult>("pepper_imitation/imitation_result", 1);

        async_spinner_.start();
        SetUpInterface(both_arms_interface_);
        ResetInterface(both_arms_interface_);

        const auto& named_targets = both_arms_interface_.getNamedTargets();
        for(const auto& target : named_targets) { ROS_ERROR("Named target: %s", target.c_str()); }
    }

    ImitationNode::~ImitationNode()
    {
        StopCheckingCurrentPose();
        ResetInterface(both_arms_interface_);
    }

    //Private
    void ImitationNode::PoseCallback(const pepper_imitation::ImitationPose& _imitation_msg)
    {
        StopCheckingCurrentPose();

        try
        {
            both_arms_interface_.setNamedTarget(pose_name_map_.at(_imitation_msg.pose));
            PlanTrajectory(both_arms_interface_);
            ExecuteTrajectory(both_arms_interface_);

            ROS_INFO("Checking pose %s for %d seconds", pose_name_map_.at(_imitation_msg.pose).c_str(), _imitation_msg.timeout);
            check_pose_ = true;
            check_pose_task_ = std::async( std::launch::async, &ImitationNode::CheckPose, this,
                                           check_poses_functions_.at(_imitation_msg.pose),
                                           std::chrono::seconds{ _imitation_msg.timeout });
        }
        catch(const std::runtime_error& ex)
        {
            ROS_ERROR("Error on pose imitation: %s", ex.what());
            SendResult(pepper_imitation::ImitationResult::ERROR);
        }
        catch(const std::out_of_range&)
        {
            ROS_ERROR("Error on pose imitation: unrecognized pose");
            SendResult(pepper_imitation::ImitationResult::ERROR);
        }
    }

    void ImitationNode::CheckPose(std::function<bool(void)> _check_pose, const std::chrono::seconds& _timeout)
    {
        const auto& start_check_time = std::chrono::system_clock::now();

        while(check_pose_)
        {
            if(std::chrono::system_clock::now() - start_check_time >= _timeout)
            {
                ROS_ERROR("Pose detection timeout");
                SendResult(pepper_imitation::ImitationResult::TIMEOUT);
                break;
            }

            try
            {
                if(_check_pose())
                {
                    ROS_ERROR("Pose correctly detected!");
                    SendResult(pepper_imitation::ImitationResult::SUCCESS);
                    break;
                }
            }
            catch(const tf::TransformException&)
            {
                ROS_ERROR("No skeleton available!");
                SendResult(pepper_imitation::ImitationResult::NO_SKELETON);
                break;
            }

            using namespace std::literals::chrono_literals;
            std::this_thread::sleep_for(100ms);
        }
    }

    void ImitationNode::StopCheckingCurrentPose()
    {
        check_pose_ = false;
        try
        {
            check_pose_task_.wait();
        }
        catch(const std::future_error&) {}
    }

    void ImitationNode::SendResult(uint8_t _result)
    {
        pepper_imitation::ImitationResult result_msg;
        result_msg.result = _result;
        imitation_result_publisher_.publish(result_msg);
    }

    void ImitationNode::SetUpInterface(PlannerInterface& _interface)
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
    }

    void ImitationNode::PlanTrajectory(PlannerInterface& _interface)
    {
        Plan movement_plan;
        const auto& result = _interface.plan(movement_plan);
        if(result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            throw std::runtime_error { std::string{ "Error planning trajectory on " } + _interface.getName() };
        }
    }

    void ImitationNode::ExecuteTrajectory(PlannerInterface& _interface)
    {
        const auto& result = _interface.asyncMove();
        if(result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            throw std::runtime_error { std::string{ "Error executing trajectory on " } + _interface.getName() };
        }
    }

    void ImitationNode::ResetInterface(PlannerInterface& _interface)
    {
        _interface.setNamedTarget("home");
        PlanTrajectory(_interface);
        ExecuteTrajectory(_interface);
    }

    bool ImitationNode::CheckHandsUpPose()
    {
        const auto& head_to_left_hand_  = GetTransform("/head_1", "/left_hand_1");
        const auto& head_to_right_hand_ = GetTransform("/head_1", "/right_hand_1");

        ROS_ERROR("HEAD TO LEFT HAND (%f,%f,%f)", head_to_left_hand_.getOrigin().x(),
                                                  head_to_left_hand_.getOrigin().y(),
                                                  head_to_left_hand_.getOrigin().z());
        return head_to_left_hand_.getOrigin().y() > 0.0 && head_to_right_hand_.getOrigin().y() > 0.0;
    }

    bool ImitationNode::CheckHandsOnHeadPose()
    {
        const auto& head_to_left_hand_  = GetTransform("/head_1", "/left_hand_1");
        const auto& head_to_right_hand_ = GetTransform("/head_1", "/right_hand_1");

        return head_to_left_hand_.getOrigin().y() > 0.0 && head_to_right_hand_.getOrigin().y() > 0.0 &&
            head_to_left_hand_.getOrigin().distance(head_to_right_hand_.getOrigin()) <= 0.2;
    }

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

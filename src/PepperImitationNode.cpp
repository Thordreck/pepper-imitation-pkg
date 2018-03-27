#include <thread>
#include <chrono>

#include "PepperImitationNode.hpp"

namespace Pepper
{
    ImitationNode::ImitationNode() :
        async_spinner_ { 1 }
    {
        node_handle_.param("velocity_scaling_factor",     velocity_scaling_factor_,     velocity_scaling_factor_);
        node_handle_.param("acceleration_scaling_factor", acceleration_scaling_factor_, acceleration_scaling_factor_);
        node_handle_.param("goal_joint_tolerance",        goal_joint_tolerance_,        goal_joint_tolerance_);
        node_handle_.param("max_planning_time_",          max_planning_time_,           max_planning_time_);
        node_handle_.param("max_planning_attempts_",      max_planning_attempts_,       max_planning_attempts_);
        node_handle_.param("motion_planner_",             motion_planner_,              motion_planner_);

        async_spinner_.start();

        SetUpInterface(arms_interface_, arms_joints_);
        SetUpInterface(head_interface_, head_joints_);
    }

    ImitationNode::~ImitationNode()
    {
        ResetInterface(arms_interface_);
        ResetInterface(head_interface_);
    }

    void ImitationNode::Loop()
    {
        MoveArms();
        using namespace std::literals::chrono_literals;
        std::this_thread::sleep_for(1s);
    }

    //Private
    void ImitationNode::MoveArms()
    {
        //arms_joints_["LShoulderPitch"] += -0.1;
        //arms_joints_["RShoulderPitch"] += -0.1;

        //arms_interface_.setJointValueTarget(arms_joints_);
        arms_interface_.setStartStateToCurrentState();
        arms_interface_.setRandomTarget();

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

    void ImitationNode::SetUpInterface(PlannerInterface& _interface, JointsMap& _joints)
    {
        _interface.startStateMonitor();

        _interface.setPlannerId(motion_planner_);
        _interface.setMaxVelocityScalingFactor(velocity_scaling_factor_);
        _interface.setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
        _interface.setGoalJointTolerance(goal_joint_tolerance_);
        _interface.setPlanningTime(max_planning_time_);
        _interface.setNumPlanningAttempts(max_planning_attempts_);

        _interface.rememberJointValues("initial_pose");
    }

    void ImitationNode::ResetInterface(PlannerInterface& _interface)
    {
        _interface.setNamedTarget("initial_pose");
        PlanTrajectory(_interface);
        ExecuteTrajectory(_interface);
    }
}

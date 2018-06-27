#include <algorithm>
#include <tf/transform_datatypes.h>
#include <pepper_imitation/ImitationResult.h>

#include "PepperImitationNode.hpp"

namespace Pepper
{
    ImitationNode::ImitationNode() :
        async_spinner_ { 1 },
        session_ { qi::makeSession() }
    {
        std::string host { "localhost" };
        int port { 9559 };

        node_handle_.param("host",             host, host);
        node_handle_.param("port",             port, port);
        node_handle_.param("skip_pose_checks", skip_pose_checks_, skip_pose_checks_);
        node_handle_.param("movement_time",    movement_time_, movement_time_);

        Connect(host, port);

        pose_subscriber_            = node_handle_.subscribe("pepper_imitation/cmd_set_pose", 1, &ImitationNode::PoseCallback, this);
        imitation_result_publisher_ = node_handle_.advertise<pepper_imitation::ImitationResult>("pepper_imitation/imitation_result", 1);

        async_spinner_.start();
        Setup();
    }

    ImitationNode::~ImitationNode()
    {
    }

    //Private
    void ImitationNode::PoseCallback(const pepper_imitation::ImitationPose& _imitation_msg)
    {
        StopCheckingCurrentPose();

        try
        {
            SetPose(poses_map_.at(_imitation_msg.pose));

            ROS_INFO("Checking pose for %d seconds", _imitation_msg.timeout);
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
        using namespace std::literals::chrono_literals;
        std::this_thread::sleep_for(3s);

        if(skip_pose_checks_)
        {
            SendResult(pepper_imitation::ImitationResult::SUCCESS);
            return;
        }

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

    void ImitationNode::SetPose(const MovementSettings& _movement_data)
    {
        for(size_t i = 0; i < _movement_data.first.size(); i++)
        {
            movement_service_.async<void>("angleInterpolation", _movement_data.first[i], _movement_data.second[i], movement_time_, true);
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

    bool ImitationNode::CheckHandsUpPose()
    {
        const auto& head_to_left_hand_  = GetTransform("/head", "/left_hand");
        const auto& head_to_right_hand_ = GetTransform("/head", "/right_hand");

        return head_to_left_hand_.getOrigin().y() > 0.0 && head_to_right_hand_.getOrigin().y() > 0.0;
    }

    bool ImitationNode::CheckHandsOnHeadPose()
    {
        const auto& head_to_left_hand  = GetTransform("/head", "/left_hand");
        const auto& head_to_right_hand = GetTransform("/head", "/right_hand");

        return head_to_left_hand.getOrigin().y() > 0.0 && head_to_right_hand.getOrigin().y() > 0.0 &&
            head_to_left_hand.getOrigin().distance(head_to_right_hand.getOrigin()) <= 0.2;
    }

    bool ImitationNode::CheckHandsOnFrontPose()
    {
        const auto& head_to_left_hand  = GetTransform("/head", "/left_hand");
        const auto& head_to_right_hand = GetTransform("/head", "/right_hand");

        return head_to_left_hand.getOrigin().y() < 0.0 && head_to_right_hand.getOrigin().y() < 0.0 &&
            head_to_left_hand.getOrigin().distance(head_to_right_hand.getOrigin()) <= 0.2;
    }

    bool ImitationNode::CheckHandsOnShoulderPose()
    {
        const auto& left_hand_to_shoulder  = GetTransform("/left_shoulder", "/left_hand");
        const auto& right_hand_to_shoulder = GetTransform("/right_shoulder", "/right_hand");

        return left_hand_to_shoulder.getOrigin().y() > 0.0 && right_hand_to_shoulder.getOrigin().y() > 0.0 &&
            left_hand_to_shoulder.getOrigin().length() <= 0.2 && right_hand_to_shoulder.getOrigin().length() <= 0.2;
    }

    bool ImitationNode::CheckCrossedArmsPose()
    {
        const auto& head_to_left_hand  = GetTransform("/head", "/left_hand");
        const auto& head_to_right_hand = GetTransform("/head", "/right_hand");

        return head_to_left_hand.getOrigin().y() < 0.0 && head_to_right_hand.getOrigin().y() < 0.0 &&
            head_to_left_hand.getOrigin().distance(head_to_right_hand.getOrigin()) <= 0.2;
    }

    bool ImitationNode::CheckHandsOnSidePose()
    {
        const auto& hip_to_left_hand  = GetTransform("/left_hip", "/left_hand");
        const auto& hip_to_right_hand = GetTransform("/right_hip", "/right_hand");

        return hip_to_left_hand.getOrigin().length() <= 0.2 && hip_to_right_hand.getOrigin().length() <= 0.2;
    }

    bool ImitationNode::CheckHandsTogetherPose()
    {
        const auto& head_to_left_hand  = GetTransform("/head", "/left_hand");
        const auto& head_to_right_hand = GetTransform("/head", "/right_hand");

        return head_to_left_hand.getOrigin().y() < 0.0 && head_to_right_hand.getOrigin().y() < 0.0 &&
            head_to_left_hand.getOrigin().distance(head_to_right_hand.getOrigin()) <= 0.2;
    }

    bool ImitationNode::CheckHandOnMouthPose()
    {
        const auto& head_to_right_hand  = GetTransform("/head", "/right_hand");
        const auto& head_to_left_hand   = GetTransform("/head", "/left_hand");

        return head_to_right_hand.getOrigin().length() <= 0.2 || head_to_left_hand.getOrigin().length() <= 0.2;
    }

    tf::StampedTransform ImitationNode::GetTransform(const std::string& _origin_frame, const std::string& _end_frame)
    {
        std::string person_id {"_"};
        std::vector<std::string> frames {};
        
        tf_listener_.getFrameStrings(frames);
        const auto& skeleton_part_it = std::find_if(frames.cbegin(), frames.cend(),
                [](const auto& frame_name){ return frame_name.rfind("torso_", 0) == 0; });

        if(skeleton_part_it != frames.cend()) { person_id += skeleton_part_it->back(); }

        tf::StampedTransform transform;
        tf_listener_.lookupTransform(_origin_frame + person_id, _end_frame + person_id, ros::Time(0), transform);

        return transform;
    }

    void ImitationNode::Connect(const std::string& _host, int _port)
    {
        try
        {
            session_->connect("tcp://" + _host + ":" + std::to_string(_port)).wait();
        }
        catch(const std::exception&)
        {
            ROS_FATAL("Error connecting to %s:%d", _host.c_str(), _port);
            ros::shutdown();
            return;
        }
        movement_service_   = session_->service("ALMotion");
        posture_service_    = session_->service("ALRobotPosture");
        autonomous_service_ = session_->service("ALAutonomousLife");
        background_service_ = session_->service("ALBackgroundMovement");
    }

    void ImitationNode::Setup()
    {
        if(autonomous_service_.call<std::string>("getState") != "disabled")
        {
            autonomous_service_.call<void>("setState", "disabled");
        }
        GoHome();
        background_service_.call<void>("setEnabled", false);
    }

    void ImitationNode::GoHome()
    {
        posture_service_.call<void>("goToPosture", "Stand", 3.0);
    }
}

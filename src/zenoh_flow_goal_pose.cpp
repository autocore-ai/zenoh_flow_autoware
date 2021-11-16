#include <array>
#include <cstdint>
#include <memory>
#include <rclcpp/node_options.hpp>
#include <zenoh_flow_goal_pose/zenoh_flow_goal_pose.hpp>
#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>
#include <iostream>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            NativeNode_goal_pose::NativeNode_goal_pose()
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                std::cout << "GoalPoseReceiver" << std::endl;
                ptr = std::make_shared<zenoh_flow::autoware_auto::goal_pose_receiver::GoalPoseReceiver>(options);
                signal(SIGINT, shutdown);
            }
            GeometryMsgsPoseStamped NativeNode_goal_pose::GetGoalPose()
            {
                return Convert(ptr->GetGoalPose());
            }
            bool NativeNode_goal_pose::IsNew()
            {
                rclcpp::spin_some(ptr);
                return ptr->IsNew();
            }
            GeometryMsgsPoseStamped get_goal_pose(std::unique_ptr<NativeNode_goal_pose> &node)
            {
                return node->GetGoalPose();
            }
            void shutdown(int sig)
            {
                (void)sig;
                exit(0);
            }
            bool is_new(std::unique_ptr<NativeNode_goal_pose> &node) { return node->IsNew(); }
            std::unique_ptr<NativeNode_goal_pose> init_goal_pose(const NativeConfig &cfg) { return std::make_unique<NativeNode_goal_pose>(); }
            std::unique_ptr<NativeNode_goal_pose> init_null_config() { return std::make_unique<NativeNode_goal_pose>(); }
        }
    }
}

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
            NativeNode::NativeNode()
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
            GeometryMsgsPoseStamped NativeNode::GetGoalPose()
            {
                return Convert(ptr->GetGoalPose());
            }
            bool NativeNode::IsNew()
            {
                rclcpp::spin_some(ptr);
                return ptr->IsNew();
            }
            GeometryMsgsPoseStamped get_goal_pose(std::unique_ptr<NativeNode> &node)
            {
                return node->GetGoalPose();
            }
            void shutdown(int sig)
            {
                (void)sig;
                exit(0);
            }
            bool is_new(std::unique_ptr<NativeNode> &node) { return node->IsNew(); }
            std::unique_ptr<NativeNode> init(const NativeConfig &cfg) { return std::make_unique<NativeNode>(); }
            std::unique_ptr<NativeNode> init_null_config() { return std::make_unique<NativeNode>(); }
        }
    }
}

#include <array>
#include <cstdint>
#include <memory>
#include <rclcpp/node_options.hpp>
#include <zenoh_flow_goal_pose/zenoh_flow_goal_pose.hpp>
#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>
namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            GoalPose::GoalPose()
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                ptr = std::make_shared<zenoh_flow::autoware_auto::goal_pose_receiver::GoalPoseReceiver>(options);
                signal(SIGINT, shutdown);
            }
            GeometryMsgsPoseStamped GoalPose::GetGoalPose()
            {
                return Convert(ptr->GetGoalPose());
            }
            bool GoalPose::IsNew()
            {
                rclcpp::spin_some(ptr);
                return ptr->IsNew();
            }
            GeometryMsgsPoseStamped goal_pose_get_goal_pose(std::unique_ptr<GoalPose> &node)
            {
                return node->GetGoalPose();
            }
            void shutdown(int sig)
            {
                (void)sig;
                exit(0);
            }
            bool goal_pose_is_new(std::unique_ptr<GoalPose> &node) { return node->IsNew(); }
            std::unique_ptr<GoalPose> goal_pose_init() { return std::make_unique<GoalPose>(); }
        }
    }
}

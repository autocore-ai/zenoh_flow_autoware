#include <zenoh_flow_goal_pose/goal_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace goal_pose_receiver
        {
            GoalPoseReceiver::GoalPoseReceiver(const rclcpp::NodeOptions &options)
                : Node("goal_pose_receiver", options)
            {
                using rclcpp::QoS;
                using std::placeholders::_1;
                sub_goal_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/planning/goal_pose", QoS{1}, std::bind(&GoalPoseReceiver::on_goalpose, this, _1));
            }

            void GoalPoseReceiver::on_goalpose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                goal_pose_msg.header = msg->header;
                goal_pose_msg.pose = msg->pose;
                is_new_msg = true;
            }

            geometry_msgs::msg::PoseStamped GoalPoseReceiver::GetGoalPose()
            {
                is_new_msg = false;
                return goal_pose_msg;
            }

            bool GoalPoseReceiver::IsNew() { return is_new_msg; }
        }
    }
}

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace goal_pose_receiver
        {
            class GoalPoseReceiver : public rclcpp::Node
            {
            public:
                explicit GoalPoseReceiver(const rclcpp::NodeOptions &options);
                geometry_msgs::msg::PoseStamped GetGoalPose();
                bool IsNew();

            private:
                rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
                geometry_msgs::msg::PoseStamped goal_pose_msg;
                bool is_new_msg = false;
                void on_goalpose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
            };
        }
    }
}

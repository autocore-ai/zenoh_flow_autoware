#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace init_pose_receiver
        {
            class InitPoseReceiver : public rclcpp::Node
            {
            public:
                explicit InitPoseReceiver(const rclcpp::NodeOptions &options);
                geometry_msgs::msg::PoseWithCovarianceStamped GetInitPose();
                bool IsNew();

            private:
                rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;
                geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;
                bool is_new_msg;
                void on_initpose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
            };
        }
    }
}

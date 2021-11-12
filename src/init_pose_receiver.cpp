#include <zenoh_flow_init_pose/init_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace init_pose_receiver
        {
            InitPoseReceiver::InitPoseReceiver(const rclcpp::NodeOptions &options)
                : Node("init_pose_receiver", options)
            {
                using rclcpp::QoS;
                using std::placeholders::_1;
                sub_init_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "/localization/initialpose", QoS{1}, std::bind(&InitPoseReceiver::on_initpose, this, _1));
            }

            void InitPoseReceiver::on_initpose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
            {
                init_pose_msg.header = msg->header;
                init_pose_msg.pose = msg->pose;
                is_new_msg = true;
            }

            geometry_msgs::msg::PoseWithCovarianceStamped InitPoseReceiver::GetInitPose()
            {
                is_new_msg = false;
                return init_pose_msg;
            }

            bool InitPoseReceiver::IsNew() { return is_new_msg; }
        }
    }
}

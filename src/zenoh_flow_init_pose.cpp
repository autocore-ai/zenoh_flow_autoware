#include <array>
#include <cstdint>
#include <memory>
#include <rclcpp/node_options.hpp>
#include <zenoh_flow_init_pose/zenoh_flow_init_pose.hpp>
#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>
#include <iostream>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            NativeNode_init_pose::NativeNode_init_pose()
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                std::cout << "InitPoseReceiver" << std::endl;
                ptr = std::make_shared<zenoh_flow::autoware_auto::init_pose_receiver::InitPoseReceiver>(options);
                signal(SIGINT, shutdown);
            }
            GeometryMsgsPoseWithCovarianceStamped NativeNode_init_pose::GetInitPose()
            {
                return Convert(ptr->GetInitPose());
            }
            bool NativeNode_init_pose::IsNew()
            {
                rclcpp::spin_some(ptr);
                return ptr->IsNew();
            }
            GeometryMsgsPoseWithCovarianceStamped get_init_pose(std::unique_ptr<NativeNode_init_pose> &node)
            {
                return node->GetInitPose();
            }
            void shutdown(int sig)
            {
                (void)sig;
                exit(0);
            }
            bool is_new_init_pose(std::unique_ptr<NativeNode_init_pose> &node) { return node->IsNew(); }
            std::unique_ptr<NativeNode_init_pose> init_init_pose(const NativeConfig &cfg) { return std::make_unique<NativeNode_init_pose>(); }
        }
    }
}

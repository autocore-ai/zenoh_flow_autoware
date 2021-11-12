#include <array>
#include <cstdint>
#include <memory>
#include <rclcpp/node_options.hpp>
#include <zenoh_flow_init_pose/zenoh_flow_init_pose.hpp>
#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>
namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            InitPose::InitPose()
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                ptr = std::make_shared<zenoh_flow::autoware_auto::init_pose_receiver::InitPoseReceiver>(options);
                signal(SIGINT, shutdown);
            }
            GeometryMsgsPoseWithCovarianceStamped InitPose::GetInitPose()
            {
                return Convert(ptr->GetInitPose());
            }
            bool InitPose::IsNew()
            {
                rclcpp::spin_some(ptr);
                return ptr->IsNew();
            }
            GeometryMsgsPoseWithCovarianceStamped init_pose_get_init_pose(std::unique_ptr<InitPose> &node)
            {
                return node->GetInitPose();
            }
            void shutdown(int sig)
            {
                (void)sig;
                exit(0);
            }
            bool init_pose_is_new(std::unique_ptr<InitPose> &node) { return node->IsNew(); }
            std::unique_ptr<InitPose> init_pose_init() { return std::make_unique<InitPose>(); }
        }
    }
}

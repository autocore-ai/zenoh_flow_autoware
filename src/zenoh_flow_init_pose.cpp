#include <zenoh_flow_init_pose/zenoh_flow_init_pose.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            GeometryMsgsPoseWithCovarianceStamped get_init_pose(std::unique_ptr<InitPose> &)
            {
                return GeometryMsgsPoseWithCovarianceStamped();
            }
            bool is_new_init_pose(std::unique_ptr<InitPose> &)
            {
                return false;
            }
            std::unique_ptr<InitPose> init_init_pose()
            {
                return std::make_unique<InitPose>();
            }
        }
    }
}

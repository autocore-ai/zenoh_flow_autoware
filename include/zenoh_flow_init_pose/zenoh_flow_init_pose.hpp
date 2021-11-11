#pragma once
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class InitPose
            {
            };
            GeometryMsgsPoseWithCovarianceStamped get_init_pose(std::unique_ptr<InitPose> &);
            bool is_new_init_pose(std::unique_ptr<InitPose> &);
            std::unique_ptr<InitPose> init_init_pose();
        }
    }
}

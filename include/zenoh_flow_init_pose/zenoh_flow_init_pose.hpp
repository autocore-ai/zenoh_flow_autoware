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
            GeometryMsgsPoseWithCovarianceStamped init_pose_get_init_pose(std::unique_ptr<InitPose> &);
            bool init_pose_is_new(std::unique_ptr<InitPose> &);
            std::unique_ptr<InitPose> init_pose_init();
        }
    }
}

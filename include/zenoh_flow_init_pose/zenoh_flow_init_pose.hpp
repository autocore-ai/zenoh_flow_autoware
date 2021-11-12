#pragma once
#include <autoware_auto.hpp>
#include <zenoh_flow_init_pose/init_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class InitPose
            {
            public:
                InitPose();
                GeometryMsgsPoseWithCovarianceStamped GetInitPose();
                bool IsNew();

            private:
                std::shared_ptr<zenoh_flow::autoware_auto::init_pose_receiver::InitPoseReceiver> ptr;
            };
            GeometryMsgsPoseWithCovarianceStamped init_pose_get_init_pose(std::unique_ptr<InitPose> &);
            bool init_pose_is_new(std::unique_ptr<InitPose> &);
            std::unique_ptr<InitPose> init_pose_init();
            void shutdown(int sig);
        }
    }
}

#pragma once
#include <msgs.hpp>
#include <zenoh_flow_init_pose.hpp>
#include <zenoh_flow_init_pose/init_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_init_pose
            {
            public:
                NativeNode_init_pose();
                GeometryMsgsPoseWithCovarianceStamped GetInitPose();
                bool IsNew();

            private:
                std::shared_ptr<zenoh_flow::autoware_auto::init_pose_receiver::InitPoseReceiver> ptr;
            };
            GeometryMsgsPoseWithCovarianceStamped get_init_pose(std::unique_ptr<NativeNode_init_pose> &);
            bool is_new_init_pose(std::unique_ptr<NativeNode_init_pose> &);
            std::unique_ptr<NativeNode_init_pose> init_init_pose(const NativeConfig &);
            void shutdown(int sig);
        }
    }
}

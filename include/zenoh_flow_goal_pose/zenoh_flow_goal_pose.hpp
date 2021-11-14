#pragma once
#include <configs.hpp>
#include <msgs.hpp>
#include <zenoh_flow_goal_pose.hpp>
#include <zenoh_flow_goal_pose/goal_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode
            {
            public:
                NativeNode();
                GeometryMsgsPoseStamped GetGoalPose();
                bool IsNew();

            private:
                std::shared_ptr<zenoh_flow::autoware_auto::goal_pose_receiver::GoalPoseReceiver> ptr;
            };
            GeometryMsgsPoseStamped get_goal_pose(std::unique_ptr<NativeNode> &);
            bool is_new(std::unique_ptr<NativeNode> &);
            std::unique_ptr<NativeNode> init(const NativeConfig &);
            std::unique_ptr<NativeNode> init_null_config();
            void shutdown(int sig);
        }
    }
}

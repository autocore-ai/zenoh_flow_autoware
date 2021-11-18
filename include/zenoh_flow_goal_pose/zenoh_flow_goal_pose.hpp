#pragma once
#include <msgs.hpp>
#include <zenoh_flow_goal_pose.hpp>
#include <zenoh_flow_goal_pose/goal_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_goal_pose
            {
            public:
                NativeNode_goal_pose();
                GeometryMsgsPoseStamped GetGoalPose();
                bool IsNew();

            private:
                std::shared_ptr<zenoh_flow::autoware_auto::goal_pose_receiver::GoalPoseReceiver> ptr;
            };
            GeometryMsgsPoseStamped get_goal_pose(std::unique_ptr<NativeNode_goal_pose> &);
            bool is_new_goal_pose(std::unique_ptr<NativeNode_goal_pose> &);
            std::unique_ptr<NativeNode_goal_pose> init_goal_pose(const NativeConfig &);
            void shutdown(int sig);
        }
    }
}

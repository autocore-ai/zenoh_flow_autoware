#pragma once
#include <autoware_auto.hpp>
#include <zenoh_flow_goal_pose/goal_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class GoalPose
            {
            public:
                GoalPose();
                GeometryMsgsPoseStamped GetGoalPose();
                bool IsNew();

            private:
                std::shared_ptr<zenoh_flow::autoware_auto::goal_pose_receiver::GoalPoseReceiver> ptr;
            };
            GeometryMsgsPoseStamped goal_pose_get_goal_pose(std::unique_ptr<GoalPose> &);
            bool goal_pose_is_new(std::unique_ptr<GoalPose> &);
            std::unique_ptr<GoalPose> goal_pose_init();
            void shutdown(int sig);
        }
    }
}

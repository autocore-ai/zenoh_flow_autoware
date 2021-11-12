#pragma once
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class GoalPose
            {
            };
            GeometryMsgsPoseStamped goal_pose_get_goal_pose(std::unique_ptr<GoalPose> &);
            bool goal_pose_is_new(std::unique_ptr<GoalPose> &);
            std::unique_ptr<GoalPose> goal_pose_init();
        }
    }
}

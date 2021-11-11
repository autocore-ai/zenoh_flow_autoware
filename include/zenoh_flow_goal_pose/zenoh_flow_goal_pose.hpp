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
            GeometryMsgsPoseStamped get_goal_pose(std::unique_ptr<GoalPose> &);
            bool is_new_goal_pose(std::unique_ptr<GoalPose> &);
            std::unique_ptr<GoalPose> init_goal_pose();
        }
    }
}

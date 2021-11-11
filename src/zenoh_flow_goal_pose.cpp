#include <zenoh_flow_goal_pose/zenoh_flow_goal_pose.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            GeometryMsgsPoseWithCovarianceStamped get_goal_pose(std::unique_ptr<GoalPose> &)
            {
                return GeometryMsgsPoseWithCovarianceStamped();
            }
            bool is_new_goal_pose(std::unique_ptr<GoalPose> &)
            {
                return false;
            }
            std::unique_ptr<GoalPose> init_goal_pose()
            {
                return std::make_unique<GoalPose>();
            }
        }
    }
}

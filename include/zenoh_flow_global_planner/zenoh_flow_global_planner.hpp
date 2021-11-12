#pragma once
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class GlobalPlanner
            {
            };
            AutowareAutoMsgsHadmapRoute global_planner_get_route(std::unique_ptr<GlobalPlanner> &);
            std::unique_ptr<GlobalPlanner> global_planner_init();
            void global_planner_set_current_pose(std::unique_ptr<GlobalPlanner> &, const AutowareAutoMsgsVehicleKinematicState &);
            void global_planner_set_goal_pose(std::unique_ptr<GlobalPlanner> &, const GeometryMsgsPoseStamped &);
        }
    }
}

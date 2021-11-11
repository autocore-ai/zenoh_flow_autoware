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
            AutowareAutoMsgsHadmapRoute get_route(std::unique_ptr<GlobalPlanner> &);
            std::unique_ptr<GlobalPlanner> init_global_planner();
            void set_current_pose(std::unique_ptr<GlobalPlanner> &, const AutowareAutoMsgsVehicleKinematicState &);
            void set_goal_pose(std::unique_ptr<GlobalPlanner> &, const GeometryMsgsPoseStamped &);
        }
    }
}

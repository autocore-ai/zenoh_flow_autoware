#pragma once
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class LocalPlanner
            {
            };
            AutowareAutoMsgsTrajectory get_trajectory(std::unique_ptr<LocalPlanner> &);
            AutowareAutoMsgsVehicleStateCommand get_state_cmd(std::unique_ptr<LocalPlanner> &);
            std::unique_ptr<LocalPlanner> init_local_planner(const CfgLocalPlanner &);
            void set_kinematic_state(std::unique_ptr<LocalPlanner> &, const AutowareAutoMsgsVehicleKinematicState &);
            void set_route(std::unique_ptr<LocalPlanner> &, const AutowareAutoMsgsHadmapRoute &);
            void set_state_report(std::unique_ptr<LocalPlanner> &, const AutowareAutoMsgsVehicleStateReport &);
        }
    }
}

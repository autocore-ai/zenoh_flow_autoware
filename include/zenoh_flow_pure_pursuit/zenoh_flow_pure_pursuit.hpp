#pragma once
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class PurePursuit
            {
            };
            AutowareAutoMsgsVehicleControlCommand pure_pursuit_get_control_cmd(std::unique_ptr<PurePursuit> &);
            std::unique_ptr<PurePursuit> pure_pursuit_init(const CfgPurePursuit &);
            void pure_pursuit_set_kinematic_state(std::unique_ptr<PurePursuit> &, const AutowareAutoMsgsVehicleKinematicState &);
            void pure_pursuit_set_trajectory(std::unique_ptr<PurePursuit> &, const AutowareAutoMsgsTrajectory &);
        }
    }
}

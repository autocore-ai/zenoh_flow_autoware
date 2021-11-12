#pragma once
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class Simulator
            {
            };
            AutowareAutoMsgsVehicleStateReport simulator_get_state_report(std::unique_ptr<Simulator> &);
            AutowareAutoMsgsVehicleKinematicState simulator_get_kinematic_state(std::unique_ptr<Simulator> &);
            std::unique_ptr<Simulator> simulator_init(const CfgSimulator &);
            void simulator_update(std::unique_ptr<Simulator> &);
            void simulator_set_state_cmd(std::unique_ptr<Simulator> &, const AutowareAutoMsgsVehicleStateCommand &);
            void simulator_set_control_cmd(std::unique_ptr<Simulator> &, const AutowareAutoMsgsVehicleControlCommand &);
            void simulator_set_init_pose(std::unique_ptr<Simulator> &, const GeometryMsgsPoseWithCovarianceStamped &);
        }
    }
}

#pragma once
#include <pure_pursuit_nodes/pure_pursuit_node.hpp>
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class PurePursuit
            {
            public:
                PurePursuit(const CfgPurePursuit &);
                AutowareAutoMsgsVehicleControlCommand GetControlCmd();
                void SetTrajectory(const AutowareAutoMsgsTrajectory &);
                void SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &);

            private:
                std::shared_ptr<autoware::motion::control::pure_pursuit_nodes::PurePursuitNode> ptr;
            };
            AutowareAutoMsgsVehicleControlCommand pure_pursuit_get_vehicle_control(std::unique_ptr<PurePursuit> &);
            std::unique_ptr<PurePursuit> pure_pursuit_init(const CfgPurePursuit &);
            void pure_pursuit_set_kinematic_state(std::unique_ptr<PurePursuit> &, const AutowareAutoMsgsVehicleKinematicState &);
            void pure_pursuit_set_trajectory(std::unique_ptr<PurePursuit> &, const AutowareAutoMsgsTrajectory &);
        }
    }
}

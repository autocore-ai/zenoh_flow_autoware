#pragma once
#include <msgs.hpp>
#include <zenoh_flow_pure_pursuit.hpp>
#include <pure_pursuit_nodes/pure_pursuit_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_pure_pursuit
            {
            public:
                NativeNode_pure_pursuit(const NativeConfig &);
                AutowareAutoMsgsVehicleControlCommand GetControlCmd();
                void SetTrajectory(const AutowareAutoMsgsTrajectory &);
                void SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &);

            private:
                std::shared_ptr<autoware::motion::control::pure_pursuit_nodes::PurePursuitNode> ptr;
            };
            AutowareAutoMsgsVehicleControlCommand get_control_cmd(std::unique_ptr<NativeNode_pure_pursuit> &);
            std::unique_ptr<NativeNode_pure_pursuit> init_pure_pursuit(const NativeConfig &);
            void set_kinematic_state(std::unique_ptr<NativeNode_pure_pursuit> &, const AutowareAutoMsgsVehicleKinematicState &);
            void set_trajectory(std::unique_ptr<NativeNode_pure_pursuit> &, const AutowareAutoMsgsTrajectory &);
        }
    }
}

#pragma once
#include <configs.hpp>
#include <msgs.hpp>
#include <zenoh_flow_pure_pursuit.hpp>
#include <pure_pursuit_nodes/pure_pursuit_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode
            {
            public:
                NativeNode();
                NativeNode(const NativeConfig &);
                AutowareAutoMsgsVehicleControlCommand GetControlCmd();
                void SetTrajectory(const AutowareAutoMsgsTrajectory &);
                void SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &);

            private:
                std::shared_ptr<autoware::motion::control::pure_pursuit_nodes::PurePursuitNode> ptr;
            };
            AutowareAutoMsgsVehicleControlCommand get_control_cmd(std::unique_ptr<NativeNode> &);
            std::unique_ptr<NativeNode> init(const NativeConfig &);
            std::unique_ptr<NativeNode> init_null_config();
            void set_kinematic_state(std::unique_ptr<NativeNode> &, const AutowareAutoMsgsVehicleKinematicState &);
            void set_trajectory(std::unique_ptr<NativeNode> &, const AutowareAutoMsgsTrajectory &);
        }
    }
}

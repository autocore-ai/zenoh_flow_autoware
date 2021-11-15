#pragma once
#include <msgs.hpp>
#include <zenoh_flow_local_planner.hpp>
#include <behavior_planner_nodes/behavior_planner_node.hpp>

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
                AutowareAutoMsgsTrajectory GetTrajectory();
                AutowareAutoMsgsVehicleStateCommand GetStateCmd();
                void SetRoute(const AutowareAutoMsgsHadmapRoute &);
                void SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &);
                void SetStateReport(const AutowareAutoMsgsVehicleStateReport &);

            private:
                std::shared_ptr<autoware::behavior_planner_nodes::BehaviorPlannerNode> ptr;
            };

            AutowareAutoMsgsTrajectory get_trajectory(std::unique_ptr<NativeNode> &);
            AutowareAutoMsgsVehicleStateCommand get_state_cmd(std::unique_ptr<NativeNode> &);
            std::unique_ptr<NativeNode> init(const NativeConfig &);
            std::unique_ptr<NativeNode> init_null_config();
            void set_kinematic_state(std::unique_ptr<NativeNode> &, const AutowareAutoMsgsVehicleKinematicState &);
            void set_route(std::unique_ptr<NativeNode> &, const AutowareAutoMsgsHadmapRoute &);
            void set_state_report(std::unique_ptr<NativeNode> &, const AutowareAutoMsgsVehicleStateReport &);
        }
    }
}

#pragma once
#include <configs.hpp>
#include <msgs.hpp>
#include <zenoh_flow_simulator.hpp>
#include <simple_planning_simulator/simple_planning_simulator_core.hpp>

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
                AutowareAutoMsgsVehicleKinematicState GetKinematicState();
                AutowareAutoMsgsVehicleStateReport GetStateReport();
                GeometryMsgsPoseStamped GetCurrentPose();
                void SetInitPose(const GeometryMsgsPoseWithCovarianceStamped &);
                void SetStateCmd(const AutowareAutoMsgsVehicleStateCommand &);
                void SetVehicleCmd(const AutowareAutoMsgsVehicleControlCommand &);
                void Update();

            private:
                std::shared_ptr<simulation::simple_planning_simulator::SimplePlanningSimulator> ptr;
            };
            AutowareAutoMsgsVehicleKinematicState get_kinematic_state(std::unique_ptr<NativeNode> &);
            AutowareAutoMsgsVehicleStateReport get_state_report(std::unique_ptr<NativeNode> &);
            std::unique_ptr<NativeNode> init(const NativeConfig &);
            std::unique_ptr<NativeNode> init_null_config();
            void set_control_cmd(std::unique_ptr<NativeNode> &, const AutowareAutoMsgsVehicleControlCommand &);
            void set_init_pose(std::unique_ptr<NativeNode> &, const GeometryMsgsPoseWithCovarianceStamped &);
            void set_state_cmd(std::unique_ptr<NativeNode> &, const AutowareAutoMsgsVehicleStateCommand &);
            void shutdown(int sig);
            void update(std::unique_ptr<NativeNode> &);
        }
    }
}

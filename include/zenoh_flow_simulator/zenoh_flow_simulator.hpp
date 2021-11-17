#pragma once
#include <msgs.hpp>
#include <zenoh_flow_simulator.hpp>
#include <simple_planning_simulator/simple_planning_simulator_core.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_simulator
            {
            public:
                NativeNode_simulator(const NativeConfig &);
                AutowareAutoMsgsVehicleKinematicState GetKinematicState();
                AutowareAutoMsgsVehicleStateReport GetStateReport();
                GeometryMsgsPoseStamped GetCurrentPose();
                void SetInitPose(const GeometryMsgsPoseWithCovarianceStamped &);
                void SetStateCmd(const AutowareAutoMsgsVehicleStateCommand &);
                void SetVehicleCmd(const AutowareAutoMsgsVehicleControlCommand &);
                void Update();
                bool IsInitialized();

            private:
                std::shared_ptr<simulation::simple_planning_simulator::SimplePlanningSimulator> ptr;
            };
            AutowareAutoMsgsVehicleKinematicState get_kinematic_state(std::unique_ptr<NativeNode_simulator> &);
            AutowareAutoMsgsVehicleStateReport get_state_report(std::unique_ptr<NativeNode_simulator> &);
            std::unique_ptr<NativeNode_simulator> init_simulator(const NativeConfig &);
            void set_control_cmd(std::unique_ptr<NativeNode_simulator> &, const AutowareAutoMsgsVehicleControlCommand &);
            void set_init_pose(std::unique_ptr<NativeNode_simulator> &, const GeometryMsgsPoseWithCovarianceStamped &);
            void set_state_cmd(std::unique_ptr<NativeNode_simulator> &, const AutowareAutoMsgsVehicleStateCommand &);
            void shutdown(int sig);
            void update(std::unique_ptr<NativeNode_simulator> &);
            bool is_initialized(std::unique_ptr<NativeNode_simulator> &);
        }
    }
}

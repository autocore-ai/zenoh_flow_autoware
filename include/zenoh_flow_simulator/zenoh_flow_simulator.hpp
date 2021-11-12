#pragma once
#include <autoware_auto.hpp>
#include <simple_planning_simulator/simple_planning_simulator_core.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class Simulator
            {
            public:
                Simulator(const CfgSimulator &);
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
            AutowareAutoMsgsVehicleKinematicState simulator_get_kinematic_state(std::unique_ptr<Simulator> &);
            AutowareAutoMsgsVehicleStateReport simulator_get_state_report(std::unique_ptr<Simulator> &);
            std::unique_ptr<Simulator> simulator_init(const CfgSimulator &);
            void shutdown(int sig);
            void simulator_set_control_cmd(std::unique_ptr<Simulator> &, const AutowareAutoMsgsVehicleControlCommand &);
            void simulator_set_init_pose(std::unique_ptr<Simulator> &, const GeometryMsgsPoseWithCovarianceStamped &);
            void simulator_set_state_cmd(std::unique_ptr<Simulator> &, const AutowareAutoMsgsVehicleStateCommand &);
            void simulator_update(std::unique_ptr<Simulator> &);
        }
    }
}

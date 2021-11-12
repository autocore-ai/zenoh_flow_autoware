#pragma once
#include <autoware_auto.hpp>
#include <behavior_planner_nodes/behavior_planner_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class LocalPlanner
            {
            public:
                LocalPlanner(const CfgLocalPlanner &);
                AutowareAutoMsgsTrajectory GetTrajectory();
                AutowareAutoMsgsVehicleStateCommand GetStateCmd();
                void SetRoute(const AutowareAutoMsgsHadmapRoute &);
                void SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &);
                void SetStateReport(const AutowareAutoMsgsVehicleStateReport &);

            private:
                std::shared_ptr<autoware::behavior_planner_nodes::BehaviorPlannerNode> ptr;
            };

            AutowareAutoMsgsTrajectory local_planner_get_trajectory(std::unique_ptr<LocalPlanner> &);
            AutowareAutoMsgsVehicleStateCommand local_planner_get_state_cmd(std::unique_ptr<LocalPlanner> &);
            std::unique_ptr<LocalPlanner> local_planner_init(const CfgLocalPlanner &);
            void local_planner_set_kinematic_state(std::unique_ptr<LocalPlanner> &, const AutowareAutoMsgsVehicleKinematicState &);
            void local_planner_set_route(std::unique_ptr<LocalPlanner> &, const AutowareAutoMsgsHadmapRoute &);
            void local_planner_set_state_report(std::unique_ptr<LocalPlanner> &, const AutowareAutoMsgsVehicleStateReport &);
        }
    }
}

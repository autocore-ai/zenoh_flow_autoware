#pragma once
#include <autoware_auto.hpp>
#include <parking_planner_nodes/parking_planner_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class ParkingPlanner
            {
            public:
                ParkingPlanner(const CfgParkingPlanner &);

            private:
                std::shared_ptr<autoware::motion::planning::parking_planner_nodes::ParkingPlannerNode> ptr;
                void spin();
            };
            std::unique_ptr<ParkingPlanner> parking_planner_init(const CfgParkingPlanner &);
            void shutdown(int sig);
        }
    }
}

#pragma once
#include <msgs.hpp>
#include <zenoh_flow_parking_planner.hpp>
#include <parking_planner_nodes/parking_planner_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_parking_planner
            {
            public:
                NativeNode_parking_planner(const NativeConfig &);

            private:
                std::shared_ptr<autoware::motion::planning::parking_planner_nodes::ParkingPlannerNode> ptr;
                void spin();
            };
            std::unique_ptr<NativeNode_parking_planner> init_parking_planner(const NativeConfig &);
            void shutdown(int sig);
        }
    }
}

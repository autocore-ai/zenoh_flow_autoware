#pragma once
#include <msgs.hpp>
#include <zenoh_flow_lane_planner.hpp>
#include <lane_planner_nodes/lane_planner_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_lane_planner
            {
            public:
                NativeNode_lane_planner();
                NativeNode_lane_planner(const NativeConfig &);

            private:
                std::shared_ptr<autoware::lane_planner_nodes::LanePlannerNode> ptr;
                void spin();
            };
            std::unique_ptr<NativeNode_lane_planner> init_lane_planner(const NativeConfig &);
            std::unique_ptr<NativeNode_lane_planner> init_null_config();
            void shutdown(int sig);
        }
    }
}

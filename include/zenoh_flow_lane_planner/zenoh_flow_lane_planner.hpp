#pragma once
#include <configs.hpp>
#include <msgs.hpp>
#include <zenoh_flow_lane_planner.hpp>
#include <lane_planner_nodes/lane_planner_node.hpp>

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

            private:
                std::shared_ptr<autoware::lane_planner_nodes::LanePlannerNode> ptr;
                void spin();
            };
            std::unique_ptr<NativeNode> init(const NativeConfig &);
            std::unique_ptr<NativeNode> init_null_config();
            void shutdown(int sig);
        }
    }
}

#pragma once
#include <autoware_auto.hpp>
#include <lane_planner_nodes/lane_planner_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class LanePlanner
            {
            public:
                LanePlanner(const CfgLanePlanner &);

            private:
                std::shared_ptr<autoware::lane_planner_nodes::LanePlannerNode> ptr;
                void spin();
            };
            std::unique_ptr<LanePlanner> lane_planner_init(const CfgLanePlanner &);
            void shutdown(int sig);
        }
    }
}

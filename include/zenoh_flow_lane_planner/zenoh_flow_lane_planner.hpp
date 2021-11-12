#pragma once
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class LanePlanner
            {
            };
            std::unique_ptr<LanePlanner> lane_planner_init(const CfgLanePlanner &);
        }
    }
}

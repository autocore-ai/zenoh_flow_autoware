#pragma once
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class ParkingPlanner
            {
            };
            std::unique_ptr<ParkingPlanner> parking_planner_init(const CfgParkingPlanner &);
        }
    }
}

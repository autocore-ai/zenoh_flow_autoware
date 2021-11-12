#pragma once
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class PcdMapLoader
            {
            };
            std::unique_ptr<PcdMapLoader> pcd_map_loader_init(const CfgPcdMapLoader &);
        }
    }
}

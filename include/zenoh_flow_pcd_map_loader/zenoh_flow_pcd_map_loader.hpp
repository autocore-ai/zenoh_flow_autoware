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
            std::unique_ptr<PcdMapLoader> init_pcd_map_loader(const CfgPcdMapLoader &);
        }
    }
}

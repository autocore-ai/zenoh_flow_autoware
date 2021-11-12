#pragma once
#include <autoware_auto.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class OsmMapLoader
            {
            };
            std::unique_ptr<OsmMapLoader> osm_map_loader_init(const CfgOsmMapLoader &);
        }
    }
}

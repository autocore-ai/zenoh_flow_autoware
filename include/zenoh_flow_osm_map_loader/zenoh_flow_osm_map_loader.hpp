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
            std::unique_ptr<OsmMapLoader> init_osm_map_loader(const CfgOsmMapLoader &);
        }
    }
}

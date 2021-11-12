#pragma once
#include <autoware_auto.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <lanelet2_map_provider/lanelet2_map_provider_node.hpp>
#include <lanelet2_map_provider/lanelet2_map_visualizer.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class OsmMapLoader
            {
            public:
                OsmMapLoader(const CfgOsmMapLoader &);

            private:
                std::shared_ptr<autoware::lanelet2_map_provider::Lanelet2MapProviderNode> ptr;
                std::shared_ptr<autoware::lanelet2_map_provider::Lanelet2MapVisualizer> ptr_viz;
                void spin();
            };
            std::unique_ptr<OsmMapLoader> osm_map_loader_init(const CfgOsmMapLoader &);
            void shutdown(int sig);
        }
    }
}

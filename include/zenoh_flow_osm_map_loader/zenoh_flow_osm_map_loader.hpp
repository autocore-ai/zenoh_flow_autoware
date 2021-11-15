#pragma once
#include <msgs.hpp>
#include <zenoh_flow_osm_map_loader.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <lanelet2_map_provider/lanelet2_map_provider_node.hpp>
#include <lanelet2_map_provider/lanelet2_map_visualizer.hpp>

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
                std::shared_ptr<autoware::lanelet2_map_provider::Lanelet2MapProviderNode> ptr;
                std::shared_ptr<autoware::lanelet2_map_provider::Lanelet2MapVisualizer> ptr_viz;
                void spin();
            };
            std::unique_ptr<NativeNode> init(const NativeConfig &);
            std::unique_ptr<NativeNode> init_null_config();
            void shutdown(int sig);
        }
    }
}

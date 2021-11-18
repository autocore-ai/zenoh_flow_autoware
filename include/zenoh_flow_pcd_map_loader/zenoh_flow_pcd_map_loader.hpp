#pragma once
#include <msgs.hpp>
#include <zenoh_flow_pcd_map_loader.hpp>
#include <ndt_nodes/map_publisher.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_pcd_map_loader
            {
            public:
                NativeNode_pcd_map_loader(const NativeConfig &);

            private:
                std::shared_ptr<autoware::localization::ndt_nodes::NDTMapPublisherNode> ptr;
                void spin();
            };
            std::unique_ptr<NativeNode_pcd_map_loader> init_pcd_map_loader(const NativeConfig &);
            void shutdown(int sig);
        }
    }
}

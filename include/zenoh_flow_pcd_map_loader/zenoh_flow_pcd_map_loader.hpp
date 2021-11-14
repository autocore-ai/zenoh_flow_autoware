#pragma once
#include <configs.hpp>
#include <msgs.hpp>
#include <zenoh_flow_pcd_map_loader.hpp>
#include <ndt_nodes/map_publisher.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode
            {
            public:
                NativeNode(const NativeConfig &);

            private:
                std::shared_ptr<autoware::localization::ndt_nodes::NDTMapPublisherNode> ptr;
                void spin();
            };
            std::unique_ptr<NativeNode> init(const NativeConfig &);
            std::unique_ptr<NativeNode> init_null_config();
            void shutdown(int sig);
        }
    }
}

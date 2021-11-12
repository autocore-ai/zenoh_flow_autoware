#pragma once
#include <autoware_auto.hpp>
#include <ndt_nodes/map_publisher.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class PcdMapLoader
            {
            public:
                PcdMapLoader(const CfgPcdMapLoader &);

            private:
                std::shared_ptr<autoware::localization::ndt_nodes::NDTMapPublisherNode> ptr;
                void spin();
            };
            std::unique_ptr<PcdMapLoader> pcd_map_loader_init(const CfgPcdMapLoader &);
            void shutdown(int sig);
        }
    }
}

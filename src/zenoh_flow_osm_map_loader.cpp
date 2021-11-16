#include <zenoh_flow_osm_map_loader/zenoh_flow_osm_map_loader.hpp>
#include <iostream>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            NativeNode_osm_map_loader::NativeNode_osm_map_loader() {}
            NativeNode_osm_map_loader::NativeNode_osm_map_loader(const NativeConfig &cfg)
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();
                paramters.push_back(rclcpp::Parameter("map_osm_file", static_cast<std::string>(cfg.map_osm_file)));
                paramters.push_back(rclcpp::Parameter("origin_offset_lat", cfg.origin_offset_lat));
                paramters.push_back(rclcpp::Parameter("origin_offset_lon", cfg.origin_offset_lon));
                paramters.push_back(rclcpp::Parameter("latitude", cfg.latitude));
                paramters.push_back(rclcpp::Parameter("longitude", cfg.longitude));
                paramters.push_back(rclcpp::Parameter("elevation", cfg.elevation));
                options.parameter_overrides(paramters);
                std::cout << "Lanelet2MapProviderNode" << std::endl;
                ptr = std::make_shared<autoware::lanelet2_map_provider::Lanelet2MapProviderNode>(options);
                std::cout << "Lanelet2MapVisualizer" << std::endl;
                ptr_viz = std::make_shared<autoware::lanelet2_map_provider::Lanelet2MapVisualizer>(options);
                std::thread{std::bind(&NativeNode_osm_map_loader::spin, this)}.detach();
                signal(SIGINT, shutdown);
            }

            void NativeNode_osm_map_loader::spin()
            {
                while (rclcpp::ok())
                {
                    rclcpp::spin_some(ptr);
                }
            }

            void shutdown(int sig)
            {
                (void)sig;
                exit(0);
            }

            std::unique_ptr<NativeNode_osm_map_loader> init_osm_map_loader(const NativeConfig &cfg)
            {
                return std::make_unique<NativeNode_osm_map_loader>(cfg);
            }
            std::unique_ptr<NativeNode_osm_map_loader> init_null_config() { return std::make_unique<NativeNode_osm_map_loader>(); }
        }
    }
}
#include <zenoh_flow_osm_map_loader/zenoh_flow_osm_map_loader.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            OsmMapLoader::OsmMapLoader(const CfgOsmMapLoader &cfg)
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();
                paramters.push_back(
                    rclcpp::Parameter("map_osm_file", static_cast<std::string>(cfg.map_osm_file)));
                paramters.push_back(rclcpp::Parameter("origin_offset_lat", cfg.origin_offset_lat));
                paramters.push_back(rclcpp::Parameter("origin_offset_lon", cfg.origin_offset_lon));
                paramters.push_back(rclcpp::Parameter("latitude", cfg.latitude));
                paramters.push_back(rclcpp::Parameter("longitude", cfg.longitude));
                paramters.push_back(rclcpp::Parameter("elevation", cfg.elevation));
                options.parameter_overrides(paramters);
                ptr = std::make_shared<autoware::lanelet2_map_provider::Lanelet2MapProviderNode>(options);
                ptr_viz = std::make_shared<autoware::lanelet2_map_provider::Lanelet2MapVisualizer>(options);
                std::thread{std::bind(&OsmMapLoader::spin, this)}.detach();
                signal(SIGINT, shutdown);
            }

            void OsmMapLoader::spin()
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

            std::unique_ptr<OsmMapLoader> initialize(const CfgOsmMapLoader &cfg)
            {
                return std::make_unique<OsmMapLoader>(cfg);
            }
        }
    }
}
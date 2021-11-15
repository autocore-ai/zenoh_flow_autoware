#include <zenoh_flow_pcd_map_loader/zenoh_flow_pcd_map_loader.hpp>
#include <iostream>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            NativeNode::NativeNode(const NativeConfig &cfg)
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();
                paramters.push_back(rclcpp::Parameter("map_pcd_file", static_cast<std::string>(cfg.map_pcd_file)));
                paramters.push_back(rclcpp::Parameter("map_yaml_file", static_cast<std::string>(cfg.map_yaml_file)));
                paramters.push_back(rclcpp::Parameter("map_frame", static_cast<std::string>(cfg.map_frame)));
                paramters.push_back(rclcpp::Parameter("map_config.capacity", cfg.map_config.capacity));
                paramters.push_back(rclcpp::Parameter("map_config.min_point.x", cfg.map_config.min_point.x));
                paramters.push_back(rclcpp::Parameter("map_config.min_point.y", cfg.map_config.min_point.y));
                paramters.push_back(rclcpp::Parameter("map_config.min_point.z", cfg.map_config.min_point.z));
                paramters.push_back(rclcpp::Parameter("map_config.max_point.x", cfg.map_config.max_point.x));
                paramters.push_back(rclcpp::Parameter("map_config.max_point.y", cfg.map_config.max_point.y));
                paramters.push_back(rclcpp::Parameter("map_config.max_point.z", cfg.map_config.max_point.z));
                paramters.push_back(rclcpp::Parameter("map_config.voxel_size.x", cfg.map_config.voxel_size.x));
                paramters.push_back(rclcpp::Parameter("map_config.voxel_size.y", cfg.map_config.voxel_size.y));
                paramters.push_back(rclcpp::Parameter("map_config.voxel_size.z", cfg.map_config.voxel_size.z));
                paramters.push_back(rclcpp::Parameter("viz_map", cfg.viz_map));
                options.parameter_overrides(paramters);
                std::cout << "NDTMapPublisherNode" << std::endl;
                ptr = std::make_shared<autoware::localization::ndt_nodes::NDTMapPublisherNode>(options);
                std::thread{std::bind(&NativeNode::spin, this)}.detach();
                signal(SIGINT, shutdown);
            }

            void NativeNode::spin()
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

            std::unique_ptr<NativeNode> init(const NativeConfig &cfg)
            {
                return std::make_unique<NativeNode>(cfg);
            }
        }
    }
}
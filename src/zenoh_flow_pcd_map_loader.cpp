// Copyright 2021 The AutoCore.AI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <zenoh_flow_pcd_map_loader/zenoh_flow_pcd_map_loader.hpp>
#include <iostream>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            NativeNode_pcd_map_loader::NativeNode_pcd_map_loader(const NativeConfig &cfg)
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
                std::thread{std::bind(&NativeNode_pcd_map_loader::spin, this)}.detach();
                signal(SIGINT, shutdown);
            }

            void NativeNode_pcd_map_loader::spin()
            {
                while (rclcpp::ok())
                {
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                    rclcpp::spin_some(ptr);
                }
            }

            void shutdown(int sig)
            {
                (void)sig;
                exit(0);
            }

            std::unique_ptr<NativeNode_pcd_map_loader> init_pcd_map_loader(const NativeConfig &cfg)
            {
                return std::make_unique<NativeNode_pcd_map_loader>(cfg);
            }
        }
    }
}
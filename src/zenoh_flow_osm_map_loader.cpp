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

#include <zenoh_flow_osm_map_loader/zenoh_flow_osm_map_loader.hpp>
#include <iostream>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
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
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                    rclcpp::spin_some(ptr);
                    rclcpp::spin_some(ptr_viz);
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
        }
    }
}
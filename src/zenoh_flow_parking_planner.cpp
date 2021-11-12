// Copyright 2021 The AutoCore.AI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <zenoh_flow_parking_planner/zenoh_flow_parking_planner.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            ParkingPlanner::ParkingPlanner(const CfgParkingPlanner &cfg)
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();
                paramters.push_back(rclcpp::Parameter("vehicle.cg_to_front_m", cfg.vehicle.cg_to_front_m));
                paramters.push_back(rclcpp::Parameter("vehicle.cg_to_rear_m", cfg.vehicle.cg_to_rear_m));
                paramters.push_back(rclcpp::Parameter("vehicle.front_corner_stiffness", cfg.vehicle.front_corner_stiffness));
                paramters.push_back(rclcpp::Parameter("vehicle.rear_corner_stiffness", cfg.vehicle.rear_corner_stiffness));
                paramters.push_back(rclcpp::Parameter("vehicle.mass_kg", cfg.vehicle.mass_kg));
                paramters.push_back(rclcpp::Parameter("vehicle.yaw_inertia_kgm2", cfg.vehicle.yaw_inertia_kgm2));
                paramters.push_back(rclcpp::Parameter("vehicle.width_m", cfg.vehicle.width_m));
                paramters.push_back(rclcpp::Parameter("vehicle.front_overhang_m", cfg.vehicle.front_overhang_m));
                paramters.push_back(rclcpp::Parameter("vehicle.rear_overhang_m", cfg.vehicle.rear_overhang_m));
                paramters.push_back(rclcpp::Parameter("optimization_weights.steering", cfg.optimization_weights.steering));
                paramters.push_back(rclcpp::Parameter("optimization_weights.throttle", cfg.optimization_weights.throttle));
                paramters.push_back(rclcpp::Parameter("optimization_weights.goal", cfg.optimization_weights.goal));
                paramters.push_back(rclcpp::Parameter("state_bounds.lower.x_m", cfg.state_bounds.lower.x_m));
                paramters.push_back(rclcpp::Parameter("state_bounds.lower.y_m", cfg.state_bounds.lower.y_m));
                paramters.push_back(rclcpp::Parameter("state_bounds.lower.velocity_mps", cfg.state_bounds.lower.velocity_mps));
                paramters.push_back(rclcpp::Parameter("state_bounds.lower.heading_rad", cfg.state_bounds.lower.heading_rad));
                paramters.push_back(rclcpp::Parameter("state_bounds.lower.steering_rad", cfg.state_bounds.lower.steering_rad));
                paramters.push_back(rclcpp::Parameter("state_bounds.upper.x_m", cfg.state_bounds.upper.x_m));
                paramters.push_back(rclcpp::Parameter("state_bounds.upper.y_m", cfg.state_bounds.upper.y_m));
                paramters.push_back(rclcpp::Parameter("state_bounds.upper.velocity_mps", cfg.state_bounds.upper.velocity_mps));
                paramters.push_back(rclcpp::Parameter("state_bounds.upper.heading_rad", cfg.state_bounds.upper.heading_rad));
                paramters.push_back(rclcpp::Parameter("state_bounds.upper.steering_rad", cfg.state_bounds.upper.steering_rad));
                paramters.push_back(rclcpp::Parameter("command_bounds.lower.steering_rate_rps", cfg.command_bounds.lower.steering_rate_rps));
                paramters.push_back(rclcpp::Parameter("command_bounds.lower.throttle_mps2", cfg.command_bounds.lower.throttle_mps2));
                paramters.push_back(rclcpp::Parameter("command_bounds.upper.steering_rate_rps", cfg.command_bounds.upper.steering_rate_rps));
                paramters.push_back(rclcpp::Parameter("command_bounds.upper.throttle_mps2", cfg.command_bounds.upper.throttle_mps2));
                options.parameter_overrides(paramters);
                ptr = std::make_shared<autoware::motion::planning::parking_planner_nodes::ParkingPlannerNode>(options);
                std::thread{std::bind(&ParkingPlanner::spin, this)}.detach();
                signal(SIGINT, shutdown);
            }

            void ParkingPlanner::spin()
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

            std::unique_ptr<ParkingPlanner> parking_planner_init(const CfgParkingPlanner &cfg)
            {
                return std::make_unique<ParkingPlanner>(cfg);
            }
        }
    }
}

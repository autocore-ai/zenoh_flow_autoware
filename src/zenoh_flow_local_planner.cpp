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

#include <zenoh_flow_local_planner/zenoh_flow_local_planner.hpp>
#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>
#include <iostream>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            NativeNode_local_planner::NativeNode_local_planner() {}
            NativeNode_local_planner::NativeNode_local_planner(const NativeConfig &cfg)
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();
                paramters.push_back(rclcpp::Parameter("enable_object_collision_estimator", cfg.enable_object_collision_estimator));
                paramters.push_back(rclcpp::Parameter("heading_weight", cfg.heading_weight));
                paramters.push_back(rclcpp::Parameter("goal_distance_thresh", cfg.goal_distance_thresh));
                paramters.push_back(rclcpp::Parameter("stop_velocity_thresh", cfg.stop_velocity_thresh));
                paramters.push_back(rclcpp::Parameter("subroute_goal_offset_lane2parking", cfg.subroute_goal_offset_lane2parking));
                paramters.push_back(rclcpp::Parameter("subroute_goal_offset_parking2lane", cfg.subroute_goal_offset_parking2lane));
                paramters.push_back(rclcpp::Parameter("vehicle.cg_to_front_m", cfg.vehicle.cg_to_front_m));
                paramters.push_back(rclcpp::Parameter("vehicle.cg_to_rear_m", cfg.vehicle.cg_to_rear_m));
                paramters.push_back(rclcpp::Parameter("vehicle.front_overhang_m", cfg.vehicle.front_overhang_m));
                paramters.push_back(rclcpp::Parameter("vehicle.rear_overhang_m", cfg.vehicle.rear_overhang_m));
                options.parameter_overrides(paramters);
                std::cout << "BehaviorPlannerNode" << std::endl;
                ptr = std::make_shared<autoware::behavior_planner_nodes::BehaviorPlannerNode>(
                    options, autocore::NodeType::ZenohFlow);
            }
            void NativeNode_local_planner::SetRoute(const AutowareAutoMsgsHadmapRoute &msg)
            {
                if (!(msg.header.stamp.nanosec == 0 && msg.header.stamp.sec == 0))
                {
                    ptr->SetRoute(Convert(msg));
                }
            }
            void NativeNode_local_planner::SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &msg)
            {
                if (!(msg.header.stamp.nanosec == 0 && msg.header.stamp.sec == 0))
                {
                    ptr->SetKinematicState(Convert(msg));
                }
            }
            void NativeNode_local_planner::SetStateReport(const AutowareAutoMsgsVehicleStateReport &msg)
            {
                if (!(msg.stamp.nanosec == 0 && msg.stamp.sec == 0))
                {
                    ptr->SetStateReport(Convert(msg));
                }
            }
            AutowareAutoMsgsTrajectory NativeNode_local_planner::GetTrajectory() { return Convert(ptr->GetTrajectory()); }
            AutowareAutoMsgsVehicleStateCommand NativeNode_local_planner::GetStateCmd()
            {
                return Convert(ptr->GetStateCmd());
            }
            std::unique_ptr<NativeNode_local_planner> init_local_planner(const NativeConfig &cfg)
            {
                return std::make_unique<NativeNode_local_planner>(cfg);
            }
            std::unique_ptr<NativeNode_local_planner> init_null_config()
            {
                return std::make_unique<NativeNode_local_planner>();
            }
            AutowareAutoMsgsTrajectory get_trajectory(std::unique_ptr<NativeNode_local_planner> &node)
            {
                return node->GetTrajectory();
            }
            AutowareAutoMsgsVehicleStateCommand get_state_cmd(std::unique_ptr<NativeNode_local_planner> &node)
            {
                return node->GetStateCmd();
            }
            void set_route(std::unique_ptr<NativeNode_local_planner> &node, const AutowareAutoMsgsHadmapRoute &msg)
            {
                node->SetRoute(msg);
            }
            void set_kinematic_state(
                std::unique_ptr<NativeNode_local_planner> &node, const AutowareAutoMsgsVehicleKinematicState &msg)
            {
                node->SetKinematicState(msg);
            }
            void set_state_report(
                std::unique_ptr<NativeNode_local_planner> &node, const AutowareAutoMsgsVehicleStateReport &msg)
            {
                node->SetStateReport(msg);
            }
        }
    }
}

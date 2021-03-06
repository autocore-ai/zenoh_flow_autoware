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

#include <zenoh_flow_pure_pursuit/zenoh_flow_pure_pursuit.hpp>
#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>
#include <iostream>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            NativeNode_pure_pursuit::NativeNode_pure_pursuit(const NativeConfig &cfg)
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                auto config = autoware::motion::control::pure_pursuit::Config(
                    cfg.minimum_lookahead_distance,
                    cfg.maximum_lookahead_distance,
                    cfg.speed_to_lookahead_ratio,
                    cfg.is_interpolate_lookahead_point,
                    cfg.is_delay_compensation,
                    cfg.emergency_stop_distance,
                    cfg.speed_thres_traveling_direction,
                    cfg.dist_front_rear_wheels);
                std::cout << "PurePursuitNode" << std::endl;
                ptr = std::make_shared<autoware::motion::control::pure_pursuit_nodes::PurePursuitNode>(
                    "pure_pursuit_node", config, "", autocore::NodeType::ZenohFlow);
            }
            AutowareAutoMsgsVehicleControlCommand NativeNode_pure_pursuit::GetControlCmd()
            {
                return Convert(ptr->GetVehicleCmd());
            }
            void NativeNode_pure_pursuit::SetTrajectory(const AutowareAutoMsgsTrajectory &msg)
            {
                ptr->SetTrajectory(Convert(msg));
                rclcpp::spin_some(this->ptr);
            }
            void NativeNode_pure_pursuit::SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &msg)
            {
                ptr->SetKinematicState(Convert(msg));
                rclcpp::spin_some(this->ptr);
            }
            AutowareAutoMsgsVehicleControlCommand get_control_cmd(std::unique_ptr<NativeNode_pure_pursuit> &node)
            {
                return node->GetControlCmd();
            }
            std::unique_ptr<NativeNode_pure_pursuit> init_pure_pursuit(const NativeConfig &cfg) { return std::make_unique<NativeNode_pure_pursuit>(cfg); }
            void set_kinematic_state(std::unique_ptr<NativeNode_pure_pursuit> &node, const AutowareAutoMsgsVehicleKinematicState &msg)
            {
                node->SetKinematicState(msg);
            }
            void set_trajectory(std::unique_ptr<NativeNode_pure_pursuit> &node, const AutowareAutoMsgsTrajectory &msg)
            {
                node->SetTrajectory(msg);
            }
        }
    }
}

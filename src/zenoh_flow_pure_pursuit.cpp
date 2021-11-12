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

namespace zenoh_flow
{
namespace autoware_auto
{
namespace ffi
{

PurePursuit::PurePursuit(const CfgPurePursuit &cfg)
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();

    paramters.push_back(
        rclcpp::Parameter("minimum_lookahead_distance", static_cast<double>(cfg.minimum_lookahead_distance)));
    paramters.push_back(
        rclcpp::Parameter("maximum_lookahead_distance", static_cast<double>(cfg.maximum_lookahead_distance)));
    paramters.push_back(
        rclcpp::Parameter("speed_to_lookahead_ratio", static_cast<double>(cfg.speed_to_lookahead_ratio)));
    paramters.push_back(
        rclcpp::Parameter("is_interpolate_lookahead_point", static_cast<bool>(cfg.is_interpolate_lookahead_point)));
    paramters.push_back(rclcpp::Parameter("is_delay_compensation", static_cast<bool>(cfg.is_delay_compensation)));
    paramters.push_back(rclcpp::Parameter("emergency_stop_distance", static_cast<double>(cfg.emergency_stop_distance)));
    paramters.push_back(rclcpp::Parameter("speed_thres_traveling_direction", static_cast<double>(cfg.speed_thres_traveling_direction)));
    paramters.push_back(rclcpp::Parameter("dist_front_rear_wheels", static_cast<double>(cfg.dist_front_rear_wheels)));

    options.parameter_overrides(paramters);

    ptr = std::make_shared<autoware::motion::control::pure_pursuit_nodes::PurePursuitNode>(
            "pure_pursuit_node", options, autocore::NodeType::ZenohFlow);
}

AutowareAutoMsgsVehicleControlCommand PurePursuit::GetVehicleCmd()
{
    return Convert(ptr->GetVehicleCmd());
}

void PurePursuit::SetTrajectory(const AutowareAutoMsgsTrajectory &msg)
{
    ptr->SetTrajectory(Convert(msg));
    rclcpp::spin_some(this->ptr);
}
void PurePursuit::SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &msg)
{
    ptr->SetKinematicState(Convert(msg));
    rclcpp::spin_some(this->ptr);
}


std::unique_ptr<PurePursuit> pure_pursuit_init(const CfgPurePursuit &cfg)
{
    return std::make_unique<PurePursuit>(cfg);
}
AutowareAutoMsgsVehicleControlCommand pure_pursuit_get_control_cmd(std::unique_ptr<PurePursuit> &node)
{
    return node->GetVehicleCmd();
}
void pure_pursuit_set_trajectory(std::unique_ptr<PurePursuit> &node, const AutowareAutoMsgsTrajectory &msg)
{
    node->SetTrajectory(msg);
}
void pure_pursuit_set_kinematic_state(
    std::unique_ptr<PurePursuit> &node, const AutowareAutoMsgsVehicleKinematicState &msg)
{
     node->SetKinematicState(msg);
}
}
} // namespace autoware_auto
} // namespace rust_cxx
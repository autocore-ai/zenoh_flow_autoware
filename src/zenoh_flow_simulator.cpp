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

#include <array>
#include <cstdint>
#include <memory>
#include <zenoh_flow_simulator/zenoh_flow_simulator.hpp>

namespace zenoh_flow
{
namespace autoware_auto
{
namespace ffi
{
Simulator::Simulator(const CfgSimulator & cfg)
{
  if(!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();

  paramters.push_back(
    rclcpp::Parameter("simulated_frame_id", static_cast<std::string>(cfg.simulated_frame_id)));
  paramters.push_back(
    rclcpp::Parameter("origin_frame_id", static_cast<std::string>(cfg.origin_frame_id)));
  paramters.push_back(
    rclcpp::Parameter("vehicle_model_type", static_cast<std::string>(cfg.vehicle_model_type)));
  paramters.push_back(
    rclcpp::Parameter("initialize_source", static_cast<std::string>(cfg.initialize_source)));
  paramters.push_back(rclcpp::Parameter("timer_sampling_time_ms", cfg.timer_sampling_time_ms));
  paramters.push_back(rclcpp::Parameter("add_measurement_noise", cfg.add_measurement_noise));
  paramters.push_back(rclcpp::Parameter("vel_lim", cfg.vel_lim));
  paramters.push_back(rclcpp::Parameter("vel_rate_lim", cfg.vel_rate_lim));
  paramters.push_back(rclcpp::Parameter("steer_lim", cfg.steer_lim));
  paramters.push_back(rclcpp::Parameter("steer_rate_lim", cfg.steer_rate_lim));
  paramters.push_back(rclcpp::Parameter("acc_time_delay", cfg.acc_time_delay));
  paramters.push_back(rclcpp::Parameter("acc_time_constant", cfg.acc_time_constant));
  paramters.push_back(rclcpp::Parameter("steer_time_delay", cfg.steer_time_delay));
  paramters.push_back(rclcpp::Parameter("steer_time_constant", cfg.steer_time_constant));

  options.parameter_overrides(paramters);
  ptr = std::make_shared<simulation::simple_planning_simulator::SimplePlanningSimulator>(
    options, ::autocore::NodeType::ZenohFlow);
  signal(SIGINT, shutdown);
}


void simulator_shutdown(int sig)
{
  (void)sig;
  exit(0);
}
AutowareAutoMsgsVehicleKinematicState Simulator::GetKinematicState()
{
  return Convert(ptr->GetKinematicState());
}
AutowareAutoMsgsVehicleStateReport Simulator::GetStateReport()
{
  return Convert(ptr->GetStateReport());
}
GeometryMsgsPoseStamped Simulator::GetCurrentPose() { return Convert(ptr->GetCurrentPose()); }

void Simulator::SetInitPose(const GeometryMsgsPoseWithCovarianceStamped & msg)
{
  ptr->SetInitPose(Convert(msg));
}
void Simulator::SetStateCmd(const AutowareAutoMsgsVehicleStateCommand & msg)
{
  ptr->SetStateCmd(Convert(msg));
}
void Simulator::SetVehicleCmd(const AutowareAutoMsgsVehicleControlCommand & msg)
{
  ptr->SetVehicleCmd(Convert(msg));
}
void Simulator::Update() { ptr->Update(); }

std::unique_ptr<Simulator> initialize(const CfgSimulator & cfg)
{
  return std::make_unique<Simulator>(cfg);
}
AutowareAutoMsgsVehicleStateCommand getKinematicState(std::unique_ptr<Simulator> & node)
{
  return node->GetKinematicState();
}
AutowareAutoMsgsVehicleStateReport getStateReport(std::unique_ptr<Simulator> & node)
{
  return node->GetStateReport();
}
GeometryMsgsPoseStamped getCurrentPose(std::unique_ptr<Simulator> & node)
{
  return node->GetCurrentPose();
}
void setInitPose(
  std::unique_ptr<Simulator> & node, const GeometryMsgsPoseWithCovarianceStamped & msg)
{
  node->SetInitPose(msg);
}
void setStateCmd(
  std::unique_ptr<Simulator> & node, const AutowareAutoMsgsVehicleStateCommand & msg)
{
  node->SetStateCmd(msg);
}
void setVehicleCmd(
  std::unique_ptr<Simulator> & node, const AutowareAutoMsgsVehicleControlCommand & msg)
{
  node->SetVehicleCmd(msg);
}
void update(std::unique_ptr<Simulator> & node) { node->Update(); }   
}
}  // namespace autoware_auto
}  // namespace rust_cxx

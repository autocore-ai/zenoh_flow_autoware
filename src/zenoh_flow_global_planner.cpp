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

#include <zenoh_flow_global_planner/zenoh_flow_global_planner.hpp>
#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>
#include <iostream>
namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            NativeNode_global_planner::NativeNode_global_planner(const NativeConfig &cfg)
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                std::cout << "Lanelet2GlobalPlannerNode" << std::endl;
                ptr = std::make_shared<autoware::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode>(
                    options, autocore::NodeType::ZenohFlow);
            }

            void NativeNode_global_planner::SetCurrentPose(const AutowareAutoMsgsVehicleKinematicState &msg)
            {
                ptr->SetCurrentPose(Convert(msg));
            }
            void NativeNode_global_planner::SetGoalPose(const GeometryMsgsPoseStamped &msg)
            {
                ptr->SetGoalPose(Convert(msg));
            }
            AutowareAutoMsgsHadmapRoute NativeNode_global_planner::GetRoute() { return Convert(ptr->GetRoute()); }
            std::unique_ptr<NativeNode_global_planner> init_global_planner(const NativeConfig &cfg)
            {
                return std::make_unique<NativeNode_global_planner>(cfg);
            }
            void set_current_pose(std::unique_ptr<NativeNode_global_planner> &node, const AutowareAutoMsgsVehicleKinematicState &msg)
            {
                node->SetCurrentPose(msg);
            }
            void set_goal_pose(std::unique_ptr<NativeNode_global_planner> &node, const GeometryMsgsPoseStamped &msg)
            {
                node->SetGoalPose(msg);
            }
            AutowareAutoMsgsHadmapRoute get_route(std::unique_ptr<NativeNode_global_planner> &node)
            {
                return node->GetRoute();
            }
        } // namespace ffi

    } // namespace autoware_auto

} // namespace zenoh_flow
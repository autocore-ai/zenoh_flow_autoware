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

#pragma once
#include <msgs.hpp>
#include <zenoh_flow_local_planner.hpp>
#include <behavior_planner_nodes/behavior_planner_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_local_planner
            {
            public:
                NativeNode_local_planner(const NativeConfig &);
                AutowareAutoMsgsTrajectory GetTrajectory();
                AutowareAutoMsgsVehicleStateCommand GetStateCmd();
                void SetRoute(const AutowareAutoMsgsHadmapRoute &);
                void SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &);
                void SetStateReport(const AutowareAutoMsgsVehicleStateReport &);

            private:
                std::shared_ptr<autoware::behavior_planner_nodes::BehaviorPlannerNode> ptr;
            };

            AutowareAutoMsgsTrajectory get_trajectory(std::unique_ptr<NativeNode_local_planner> &);
            AutowareAutoMsgsVehicleStateCommand get_state_cmd(std::unique_ptr<NativeNode_local_planner> &);
            std::unique_ptr<NativeNode_local_planner> init_local_planner(const NativeConfig &);
            void set_kinematic_state(std::unique_ptr<NativeNode_local_planner> &, const AutowareAutoMsgsVehicleKinematicState &);
            void set_route(std::unique_ptr<NativeNode_local_planner> &, const AutowareAutoMsgsHadmapRoute &);
            void set_state_report(std::unique_ptr<NativeNode_local_planner> &, const AutowareAutoMsgsVehicleStateReport &);
        }
    }
}

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
#include <zenoh_flow_pure_pursuit.hpp>
#include <pure_pursuit_nodes/pure_pursuit_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_pure_pursuit
            {
            public:
                NativeNode_pure_pursuit(const NativeConfig &);
                AutowareAutoMsgsVehicleControlCommand GetControlCmd();
                void SetTrajectory(const AutowareAutoMsgsTrajectory &);
                void SetKinematicState(const AutowareAutoMsgsVehicleKinematicState &);

            private:
                std::shared_ptr<autoware::motion::control::pure_pursuit_nodes::PurePursuitNode> ptr;
            };
            AutowareAutoMsgsVehicleControlCommand get_control_cmd(std::unique_ptr<NativeNode_pure_pursuit> &);
            std::unique_ptr<NativeNode_pure_pursuit> init_pure_pursuit(const NativeConfig &);
            void set_kinematic_state(std::unique_ptr<NativeNode_pure_pursuit> &, const AutowareAutoMsgsVehicleKinematicState &);
            void set_trajectory(std::unique_ptr<NativeNode_pure_pursuit> &, const AutowareAutoMsgsTrajectory &);
        }
    }
}

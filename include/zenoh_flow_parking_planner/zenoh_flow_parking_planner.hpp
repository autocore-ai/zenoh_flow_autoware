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
#include <zenoh_flow_parking_planner.hpp>
#include <parking_planner_nodes/parking_planner_node.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_parking_planner
            {
            public:
                NativeNode_parking_planner(const NativeConfig &);

            private:
                std::shared_ptr<autoware::motion::planning::parking_planner_nodes::ParkingPlannerNode> ptr;
                void spin();
            };
            std::unique_ptr<NativeNode_parking_planner> init_parking_planner(const NativeConfig &);
            void shutdown(int sig);
        }
    }
}

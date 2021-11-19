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
#include <zenoh_flow_pcd_map_loader.hpp>
#include <ndt_nodes/map_publisher.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_pcd_map_loader
            {
            public:
                NativeNode_pcd_map_loader(const NativeConfig &);

            private:
                std::shared_ptr<autoware::localization::ndt_nodes::NDTMapPublisherNode> ptr;
                void spin();
            };
            std::unique_ptr<NativeNode_pcd_map_loader> init_pcd_map_loader(const NativeConfig &);
            void shutdown(int sig);
        }
    }
}

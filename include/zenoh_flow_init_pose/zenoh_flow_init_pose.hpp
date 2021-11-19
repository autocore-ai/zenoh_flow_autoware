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
#include <zenoh_flow_init_pose.hpp>
#include <zenoh_flow_init_pose/init_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_init_pose
            {
            public:
                NativeNode_init_pose();
                GeometryMsgsPoseWithCovarianceStamped GetInitPose();
                bool IsNew();

            private:
                std::shared_ptr<zenoh_flow::autoware_auto::init_pose_receiver::InitPoseReceiver> ptr;
            };
            GeometryMsgsPoseWithCovarianceStamped get_init_pose(std::unique_ptr<NativeNode_init_pose> &);
            bool is_new_init_pose(std::unique_ptr<NativeNode_init_pose> &);
            std::unique_ptr<NativeNode_init_pose> init_init_pose(const NativeConfig &);
            void shutdown(int sig);
        }
    }
}

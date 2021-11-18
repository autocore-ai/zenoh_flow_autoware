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
#include <zenoh_flow_goal_pose.hpp>
#include <zenoh_flow_goal_pose/goal_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeNode_goal_pose
            {
            public:
                NativeNode_goal_pose();
                GeometryMsgsPoseStamped GetGoalPose();
                bool IsNew();

            private:
                std::shared_ptr<zenoh_flow::autoware_auto::goal_pose_receiver::GoalPoseReceiver> ptr;
            };
            GeometryMsgsPoseStamped get_goal_pose(std::unique_ptr<NativeNode_goal_pose> &);
            bool is_new_goal_pose(std::unique_ptr<NativeNode_goal_pose> &);
            std::unique_ptr<NativeNode_goal_pose> init_goal_pose(const NativeConfig &);
            void shutdown(int sig);
        }
    }
}

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

#include <array>
#include <cstdint>
#include <memory>
#include <rclcpp/node_options.hpp>
#include <zenoh_flow_init_pose/zenoh_flow_init_pose.hpp>
#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>
#include <iostream>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            NativeNode_init_pose::NativeNode_init_pose()
            {
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }
                rclcpp::NodeOptions options;
                std::cout << "InitPoseReceiver" << std::endl;
                ptr = std::make_shared<zenoh_flow::autoware_auto::init_pose_receiver::InitPoseReceiver>(options);
                signal(SIGINT, shutdown);
            }
            GeometryMsgsPoseWithCovarianceStamped NativeNode_init_pose::GetInitPose()
            {
                return Convert(ptr->GetInitPose());
            }
            bool NativeNode_init_pose::IsNew()
            {
                rclcpp::spin_some(ptr);
                return ptr->IsNew();
            }
            GeometryMsgsPoseWithCovarianceStamped get_init_pose(std::unique_ptr<NativeNode_init_pose> &node)
            {
                return node->GetInitPose();
            }
            void shutdown(int sig)
            {
                (void)sig;
                exit(0);
            }
            bool is_new_init_pose(std::unique_ptr<NativeNode_init_pose> &node) { return node->IsNew(); }
            std::unique_ptr<NativeNode_init_pose> init_init_pose(const NativeConfig &cfg) { return std::make_unique<NativeNode_init_pose>(); }
        }
    }
}

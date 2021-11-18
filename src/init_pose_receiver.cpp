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

#include <zenoh_flow_init_pose/init_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace init_pose_receiver
        {
            InitPoseReceiver::InitPoseReceiver(const rclcpp::NodeOptions &options)
                : Node("init_pose_receiver", options)
            {
                using rclcpp::QoS;
                using std::placeholders::_1;
                sub_init_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "/localization/initialpose", QoS{1}, std::bind(&InitPoseReceiver::on_initpose, this, _1));
            }

            void InitPoseReceiver::on_initpose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
            {
                init_pose_msg.header = msg->header;
                init_pose_msg.pose = msg->pose;
                is_new_msg = true;
            }

            geometry_msgs::msg::PoseWithCovarianceStamped InitPoseReceiver::GetInitPose()
            {
                is_new_msg = false;
                return init_pose_msg;
            }

            bool InitPoseReceiver::IsNew() { return is_new_msg; }
        }
    }
}

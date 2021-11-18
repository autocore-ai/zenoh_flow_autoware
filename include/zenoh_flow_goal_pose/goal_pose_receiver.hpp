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
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace goal_pose_receiver
        {
            class GoalPoseReceiver : public rclcpp::Node
            {
            public:
                explicit GoalPoseReceiver(const rclcpp::NodeOptions &options);
                geometry_msgs::msg::PoseStamped GetGoalPose();
                bool IsNew();

            private:
                rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
                geometry_msgs::msg::PoseStamped goal_pose_msg;
                bool is_new_msg = false;
                void on_goalpose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
            };
        }
    }
}

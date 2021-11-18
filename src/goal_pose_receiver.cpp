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

#include <zenoh_flow_goal_pose/goal_pose_receiver.hpp>

namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace goal_pose_receiver
        {
            GoalPoseReceiver::GoalPoseReceiver(const rclcpp::NodeOptions &options)
                : Node("goal_pose_receiver", options)
            {
                using rclcpp::QoS;
                using std::placeholders::_1;
                sub_goal_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/planning/goal_pose", QoS{1}, std::bind(&GoalPoseReceiver::on_goalpose, this, _1));
            }

            void GoalPoseReceiver::on_goalpose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                goal_pose_msg.header = msg->header;
                goal_pose_msg.pose = msg->pose;
                is_new_msg = true;
            }

            geometry_msgs::msg::PoseStamped GoalPoseReceiver::GetGoalPose()
            {
                is_new_msg = false;
                return goal_pose_msg;
            }

            bool GoalPoseReceiver::IsNew() { return is_new_msg; }
        }
    }
}

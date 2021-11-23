# Copyright 2021 The AutoCore.AI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/bin/sh

set -e

rustdoc(){
    cargo rustdoc -p $1 --target-dir docs
}

rustdoc "autoware_auto"
rustdoc "common"
rustdoc "derive"
rustdoc "zenoh_flow_global_planner"
rustdoc "zenoh_flow_goal_pose"
rustdoc "zenoh_flow_init_pose"
rustdoc "zenoh_flow_lane_planner"
rustdoc "zenoh_flow_local_planner"
rustdoc "zenoh_flow_osm_map_loader"
rustdoc "zenoh_flow_parking_planner"
rustdoc "zenoh_flow_pcd_map_loader"
rustdoc "zenoh_flow_pure_pursuit"
rustdoc "zenoh_flow_simulator"
rustdoc "zenoh_flow_system_monitor"
rustdoc "zenoh_flow_tick_source"

rm docs/doc/.lock

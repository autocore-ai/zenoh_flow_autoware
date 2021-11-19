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

use cxx::UniquePtr;
use std::{
    any::type_name,
    fmt::{Debug, Formatter, Result},
};
use zenoh_flow::zenoh_flow_derive::ZFState;

/// CXX binding functions for goal pose
#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    #[derive(DefaultConfig)]
    struct NativeConfig {
        pub node_name: String,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_goal_pose/zenoh_flow_goal_pose.hpp");
        type NativeNode_goal_pose;

        type GeometryMsgsPoseStamped = autoware_auto::msgs::ffi::GeometryMsgsPoseStamped;

        fn init_goal_pose(cfg: &NativeConfig) -> UniquePtr<NativeNode_goal_pose>;
        fn get_goal_pose(node: &mut UniquePtr<NativeNode_goal_pose>) -> GeometryMsgsPoseStamped;
        fn is_new_goal_pose(node: &mut UniquePtr<NativeNode_goal_pose>) -> bool;
    }
}
unsafe impl Send for ffi::NativeNode_goal_pose {}
unsafe impl Sync for ffi::NativeNode_goal_pose {}

impl Debug for ffi::NativeNode_goal_pose {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_goal_pose>())
            .finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_goal_pose>,
}

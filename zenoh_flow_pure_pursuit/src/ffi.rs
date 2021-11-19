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

#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    struct NativeConfig {
        pub minimum_lookahead_distance: f64,
        pub maximum_lookahead_distance: f64,
        pub speed_to_lookahead_ratio: f64,
        pub is_interpolate_lookahead_point: bool,
        pub is_delay_compensation: bool,
        pub emergency_stop_distance: f64,
        pub speed_thres_traveling_direction: f64,
        pub dist_front_rear_wheels: f64,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_pure_pursuit/zenoh_flow_pure_pursuit.hpp");
        type NativeNode_pure_pursuit;
        type AutowareAutoMsgsTrajectory = autoware_auto::msgs::ffi::AutowareAutoMsgsTrajectory;
        type AutowareAutoMsgsVehicleKinematicState =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleKinematicState;
        type AutowareAutoMsgsVehicleControlCommand =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleControlCommand;

        fn init_pure_pursuit(cfg: &NativeConfig) -> UniquePtr<NativeNode_pure_pursuit>;
        fn set_trajectory(
            node: &mut UniquePtr<NativeNode_pure_pursuit>,
            msg: &AutowareAutoMsgsTrajectory,
        );
        fn set_kinematic_state(
            node: &mut UniquePtr<NativeNode_pure_pursuit>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn get_control_cmd(
            node: &mut UniquePtr<NativeNode_pure_pursuit>,
        ) -> AutowareAutoMsgsVehicleControlCommand;
    }
}

unsafe impl Send for ffi::NativeNode_pure_pursuit {}
unsafe impl Sync for ffi::NativeNode_pure_pursuit {}

impl Debug for ffi::NativeNode_pure_pursuit {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_pure_pursuit>())
            .finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_pure_pursuit>,
}

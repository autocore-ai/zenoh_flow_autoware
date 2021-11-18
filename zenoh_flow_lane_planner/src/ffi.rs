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
    pub struct Vehicle {
        pub cg_to_front_m: f64,
        pub cg_to_rear_m: f64,
        pub front_corner_stiffness: f64,
        pub rear_corner_stiffness: f64,
        pub mass_kg: f64,
        pub yaw_inertia_kgm2: f64,
        pub width_m: f64,
        pub front_overhang_m: f64,
        pub rear_overhang_m: f64,
    }
    pub struct LanePlanner {
        pub trajectory_resolution: f64,
    }
    pub struct GaussianSmoother {
        pub standard_deviation: f64,
        pub kernel_size: i64,
    }
    struct NativeConfig {
        pub heading_weight: f64,
        pub lane_planner: LanePlanner,
        pub vehicle: Vehicle,
        pub gaussian_smoother: GaussianSmoother,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_lane_planner/zenoh_flow_lane_planner.hpp");
        type NativeNode_lane_planner;

        fn init_lane_planner(cfg: &NativeConfig) -> UniquePtr<NativeNode_lane_planner>;
    }
}



unsafe impl Send for ffi::NativeNode_lane_planner {}
unsafe impl Sync for ffi::NativeNode_lane_planner {}

impl Debug for ffi::NativeNode_lane_planner {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_lane_planner>()).finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_lane_planner>,
}

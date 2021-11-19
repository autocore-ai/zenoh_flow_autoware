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
    #[derive(Debug)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    #[derive(Debug)]
    pub struct MapConfig {
        pub capacity: i64,
        pub min_point: Vector3,
        pub max_point: Vector3,
        pub voxel_size: Vector3,
    }
    struct NativeConfig {
        pub map_pcd_file: String,
        pub map_yaml_file: String,
        pub map_frame: String,
        pub map_config: MapConfig,
        pub viz_map: bool,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_pcd_map_loader/zenoh_flow_pcd_map_loader.hpp");
        type NativeNode_pcd_map_loader;

        fn init_pcd_map_loader(cfg: &NativeConfig) -> UniquePtr<NativeNode_pcd_map_loader>;
    }
}

unsafe impl Send for ffi::NativeNode_pcd_map_loader {}
unsafe impl Sync for ffi::NativeNode_pcd_map_loader {}

impl Debug for ffi::NativeNode_pcd_map_loader {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_pcd_map_loader>())
            .finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_pcd_map_loader>,
}

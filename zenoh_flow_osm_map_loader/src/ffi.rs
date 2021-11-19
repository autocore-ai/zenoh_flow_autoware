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

/// CXX binding functions for osm map loader
#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    struct NativeConfig {
        pub map_osm_file: String,
        pub origin_offset_lat: f64,
        pub origin_offset_lon: f64,
        pub latitude: f64,
        pub longitude: f64,
        pub elevation: f64,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_osm_map_loader/zenoh_flow_osm_map_loader.hpp");
        type NativeNode_osm_map_loader;

        fn init_osm_map_loader(cfg: &NativeConfig) -> UniquePtr<NativeNode_osm_map_loader>;
    }
}

unsafe impl Send for ffi::NativeNode_osm_map_loader {}
unsafe impl Sync for ffi::NativeNode_osm_map_loader {}

impl Debug for ffi::NativeNode_osm_map_loader {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_osm_map_loader>())
            .finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_osm_map_loader>,
}

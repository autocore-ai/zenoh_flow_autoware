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
        f.debug_struct(type_name::<ffi::NativeNode_pcd_map_loader>()).finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_pcd_map_loader>,
}

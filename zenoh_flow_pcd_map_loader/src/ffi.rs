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
        type NativeNode = autoware_auto::ffi::NativeNode;

        fn init(cfg: &NativeConfig) -> UniquePtr<NativeNode>;
    }
}

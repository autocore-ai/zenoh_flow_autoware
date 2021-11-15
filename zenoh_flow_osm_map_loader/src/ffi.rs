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
        type NativeNode = autoware_auto::ffi::NativeNode;

        fn init(cfg: &NativeConfig) -> UniquePtr<NativeNode>;
    }
}

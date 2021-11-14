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
    pub struct LanePlannerConfig {
        pub trajectory_resolution: f64,
    }
    pub struct GaussianSmoother {
        pub standard_deviation: f64,
        pub kernel_size: i64,
    }
    struct NativeConfig {
        pub heading_weight: f64,
        pub lane_planner: LanePlannerConfig,
        pub vehicle: Vehicle,
        pub gaussian_smoother: GaussianSmoother,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_lane_planner/zenoh_flow_lane_planner.hpp");
        type NativeNode = autoware_auto::ffi::NativeNode;

        fn init(cfg: &NativeConfig) -> UniquePtr<NativeNode>;
    }
}

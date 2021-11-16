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
    pub struct OptimizationWeights {
        pub steering: f64,
        pub throttle: f64,
        pub goal: f64,
    }
    pub struct StateBound {
        pub x_m: f64,
        pub y_m: f64,
        pub velocity_mps: f64,
        pub heading_rad: f64,
        pub steering_rad: f64,
    }
    pub struct StateBounds {
        pub lower: StateBound,
        pub upper: StateBound,
    }
    pub struct CommandBound {
        pub steering_rate_rps: f64,
        pub throttle_mps2: f64,
    }
    pub struct CommandBounds {
        pub lower: CommandBound,
        pub upper: CommandBound,
    }
    struct NativeConfig {
        pub vehicle: Vehicle,
        pub optimization_weights: OptimizationWeights,
        pub state_bounds: StateBounds,
        pub command_bounds: CommandBounds,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_parking_planner/zenoh_flow_parking_planner.hpp");
        type NativeNode_parking_planner;

        fn init_parking_planner(cfg: &NativeConfig) -> UniquePtr<NativeNode_parking_planner>;
    }
}

unsafe impl Send for ffi::NativeNode_parking_planner {}
unsafe impl Sync for ffi::NativeNode_parking_planner {}

impl Debug for ffi::NativeNode_parking_planner {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_parking_planner>()).finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_parking_planner>,
}

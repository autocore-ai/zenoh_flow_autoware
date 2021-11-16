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
    struct NativeConfig {
        pub enable_object_collision_estimator: bool,
        pub heading_weight: f64,
        pub goal_distance_thresh: f64,
        pub stop_velocity_thresh: f64,
        pub subroute_goal_offset_lane2parking: f64,
        pub subroute_goal_offset_parking2lane: f64,
        pub vehicle: Vehicle,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_local_planner/zenoh_flow_local_planner.hpp");
        type NativeNode_local_planner;
        type AutowareAutoMsgsHadmapRoute = autoware_auto::msgs::ffi::AutowareAutoMsgsHadmapRoute;
        type AutowareAutoMsgsVehicleKinematicState =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleKinematicState;
        type AutowareAutoMsgsVehicleStateReport =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleStateReport;
        type AutowareAutoMsgsTrajectory = autoware_auto::msgs::ffi::AutowareAutoMsgsTrajectory;
        type AutowareAutoMsgsVehicleStateCommand =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleStateCommand;

        fn init_local_planner(cfg: &NativeConfig) -> UniquePtr<NativeNode_local_planner>;
        fn set_route(node: &mut UniquePtr<NativeNode_local_planner>, msg: &AutowareAutoMsgsHadmapRoute);
        fn set_kinematic_state(
            node: &mut UniquePtr<NativeNode_local_planner>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn set_state_report(
            node: &mut UniquePtr<NativeNode_local_planner>,
            msg: &AutowareAutoMsgsVehicleStateReport,
        );
        fn get_trajectory(node: &mut UniquePtr<NativeNode_local_planner>) -> AutowareAutoMsgsTrajectory;
        fn get_state_cmd(node: &mut UniquePtr<NativeNode_local_planner>) -> AutowareAutoMsgsVehicleStateCommand;
    }
}

unsafe impl Send for ffi::NativeNode_local_planner {}
unsafe impl Sync for ffi::NativeNode_local_planner {}

impl Debug for ffi::NativeNode_local_planner {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_local_planner>()).finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_local_planner>,
}

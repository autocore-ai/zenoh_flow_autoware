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
        type NativeNode = autoware_auto::ffi::NativeNode;
        type AutowareAutoMsgsHadmapRoute = autoware_auto::msgs::ffi::AutowareAutoMsgsHadmapRoute;
        type AutowareAutoMsgsVehicleKinematicState =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleKinematicState;
        type AutowareAutoMsgsVehicleStateReport =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleStateReport;
        type AutowareAutoMsgsTrajectory = autoware_auto::msgs::ffi::AutowareAutoMsgsTrajectory;
        type AutowareAutoMsgsVehicleStateCommand =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleStateCommand;

        fn init(cfg: &NativeConfig) -> UniquePtr<NativeNode>;
        fn set_route(node: &mut UniquePtr<NativeNode>, msg: &AutowareAutoMsgsHadmapRoute);
        fn set_kinematic_state(
            node: &mut UniquePtr<NativeNode>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn set_state_report(
            node: &mut UniquePtr<NativeNode>,
            msg: &AutowareAutoMsgsVehicleStateReport,
        );
        fn get_trajectory(node: &mut UniquePtr<NativeNode>) -> AutowareAutoMsgsTrajectory;
        fn get_state_cmd(node: &mut UniquePtr<NativeNode>) -> AutowareAutoMsgsVehicleStateCommand;
    }
}

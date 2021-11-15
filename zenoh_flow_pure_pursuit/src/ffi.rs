#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    struct NativeConfig {
        pub minimum_lookahead_distance: f64,
        pub maximum_lookahead_distance: f64,
        pub speed_to_lookahead_ratio: f64,
        pub is_interpolate_lookahead_point: bool,
        pub is_delay_compensation: bool,
        pub emergency_stop_distance: f64,
        pub speed_thres_traveling_direction: f64,
        pub dist_front_rear_wheels: f64,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_pure_pursuit/zenoh_flow_pure_pursuit.hpp");
        type NativeNode = autoware_auto::ffi::NativeNode;
        type AutowareAutoMsgsTrajectory = autoware_auto::msgs::ffi::AutowareAutoMsgsTrajectory;
        type AutowareAutoMsgsVehicleKinematicState =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleKinematicState;
        type AutowareAutoMsgsVehicleControlCommand =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleControlCommand;

        fn init(cfg: &NativeConfig) -> UniquePtr<NativeNode>;
        fn set_trajectory(node: &mut UniquePtr<NativeNode>, msg: &AutowareAutoMsgsTrajectory);
        fn set_kinematic_state(
            node: &mut UniquePtr<NativeNode>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn get_control_cmd(
            node: &mut UniquePtr<NativeNode>,
        ) -> AutowareAutoMsgsVehicleControlCommand;
    }
}

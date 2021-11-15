#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    struct NativeConfig {
        pub simulated_frame_id: String,
        pub origin_frame_id: String,
        pub vehicle_model_type: String,
        pub initialize_source: String,
        pub timer_sampling_time_ms: i64,
        pub add_measurement_noise: bool,
        pub vel_lim: f64,
        pub vel_rate_lim: f64,
        pub steer_lim: f64,
        pub steer_rate_lim: f64,
        pub acc_time_delay: f64,
        pub acc_time_constant: f64,
        pub steer_time_delay: f64,
        pub steer_time_constant: f64,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_simulator/zenoh_flow_simulator.hpp");
        type NativeNode = autoware_auto::ffi::NativeNode;
        type GeometryMsgsPoseWithCovarianceStamped =
            autoware_auto::msgs::ffi::GeometryMsgsPoseWithCovarianceStamped;
        type AutowareAutoMsgsVehicleControlCommand =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleControlCommand;
        type AutowareAutoMsgsVehicleStateCommand =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleStateCommand;
        type AutowareAutoMsgsVehicleKinematicState =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleKinematicState;
        type AutowareAutoMsgsVehicleStateReport =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleStateReport;

        fn init(cfg: &NativeConfig) -> UniquePtr<NativeNode>;
        fn set_init_pose(
            node: &mut UniquePtr<NativeNode>,
            msg: &GeometryMsgsPoseWithCovarianceStamped,
        );
        fn set_control_cmd(
            node: &mut UniquePtr<NativeNode>,
            msg: &AutowareAutoMsgsVehicleControlCommand,
        );
        fn set_state_cmd(
            node: &mut UniquePtr<NativeNode>,
            msg: &AutowareAutoMsgsVehicleStateCommand,
        );
        fn get_kinematic_state(
            node: &mut UniquePtr<NativeNode>,
        ) -> AutowareAutoMsgsVehicleKinematicState;
        fn get_state_report(node: &mut UniquePtr<NativeNode>)
            -> AutowareAutoMsgsVehicleStateReport;
        fn update(node: &mut UniquePtr<NativeNode>);
    }
}

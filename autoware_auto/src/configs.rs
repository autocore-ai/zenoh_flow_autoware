#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    #[derive(Debug)]
    pub struct CfgPurePursuit {
        pub minimum_lookahead_distance: f64,
        pub maximum_lookahead_distance: f64,
        pub speed_to_lookahead_ratio: f64,
        pub is_interpolate_lookahead_point: bool,
        pub is_delay_compensation: bool,
        pub emergency_stop_distance: f64,
        pub speed_thres_traveling_direction: f64,
        pub distance_front_rear_wheel: f64,
    }
    #[derive(Debug)]
    pub struct CfgSimulator {
        pub simulated_frame_id: String,
        pub origin_frame_id: String,
        pub vehicle_model_type: String,
        pub initialize_source: String,
        pub timer_sampling_time_ms: i32,
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
}

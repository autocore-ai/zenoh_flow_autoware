#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    #[derive(Debug)]
    pub struct CfgOsmMapLoader {
        pub map_osm_file: String,
        pub origin_offset_lat: f64,
        pub origin_offset_lon: f64,
        pub latitude: f64,
        pub longitude: f64,
        pub elevation: f64,
    }
    #[derive(Debug)]
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
    #[derive(Debug)]
    pub struct CfgLocalPlanner {
        pub enable_object_collision_estimator: bool,
        pub heading_weight: f64,
        pub goal_distance_thresh: f64,
        pub stop_velocity_thresh: f64,
        pub subroute_goal_offset_lane2parking: f64,
        pub subroute_goal_offset_parking2lane: f64,
        pub vehicle: Vehicle,
    }
    #[derive(Debug)]
    pub struct OptimizationWeights {
        pub steering: f64,
        pub throttle: f64,
        pub goal: f64,
    }
    #[derive(Debug)]
    pub struct StateBound {
        pub x_m: f64,
        pub y_m: f64,
        pub velocity_mps: f64,
        pub heading_rad: f64,
        pub steering_rad: f64,
    }
    #[derive(Debug)]
    pub struct StateBounds {
        pub lower: StateBound,
        pub upper: StateBound,
    }
    #[derive(Debug)]
    pub struct CommandBound {
        pub steering_rate_rps: f64,
        pub throttle_mps2: f64,
    }
    #[derive(Debug)]
    pub struct CommandBounds {
        pub lower: CommandBound,
        pub upper: CommandBound,
    }
    #[derive(Debug)]
    pub struct CfgParkingPlanner {
        pub vehicle: Vehicle,
        pub optimization_weights: OptimizationWeights,
        pub state_bounds: StateBounds,
        pub command_bounds: CommandBounds,
    }
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

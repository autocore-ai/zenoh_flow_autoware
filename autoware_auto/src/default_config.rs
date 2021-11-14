use crate::configs::ffi::*;

impl Default for CfgPurePursuit {
    fn default() -> Self {
        CfgPurePursuit {
            minimum_lookahead_distance: 6.0,
            maximum_lookahead_distance: 100.0,
            speed_to_lookahead_ratio: 2.0,
            is_interpolate_lookahead_point: true,
            is_delay_compensation: false,
            emergency_stop_distance: 0.1,
            speed_thres_traveling_direction: 0.3,
            distance_front_rear_wheel: 2.7,
        }
    }
}

impl Default for Vehicle {
    fn default() -> Self {
        Vehicle {
            cg_to_front_m: 1.228,
            cg_to_rear_m: 1.5618,
            front_corner_stiffness: 0.1,
            rear_corner_stiffness: 0.1,
            mass_kg: 1500.0,
            yaw_inertia_kgm2: 12.0,
            width_m: 2.0,
            front_overhang_m: 1.05,
            rear_overhang_m: 0.92,
        }
    }
}

impl Default for CfgLocalPlanner {
    fn default() -> Self {
        CfgLocalPlanner {
            enable_object_collision_estimator: false,
            heading_weight: 0.1,
            goal_distance_thresh: 3.0,
            stop_velocity_thresh: 2.0,
            subroute_goal_offset_lane2parking: 7.5,
            subroute_goal_offset_parking2lane: 7.5,
            vehicle: Vehicle::default(),
        }
    }
}

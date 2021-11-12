use crate::autoware_auto::ffi::*;

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

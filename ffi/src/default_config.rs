use crate::autoware_auto::ffi::*;

impl Default for CfgPcdMapLoader {
    fn default() -> Self {
        CfgPcdMapLoader {
            map_pcd_file: String::from(
                "/opt/AutowareAuto/share/autoware_demos/data/autonomoustuff_parking_lot_lgsvl.pcd",
            ),
            map_yaml_file: String::from(
                "/opt/AutowareAuto/share/autoware_demos/data/autonomoustuff_parking_lot_lgsvl.yaml",
            ),
            map_frame: String::from("map"),
            map_config: MapConfig {
                capacity: 1000000,
                min_point: Vector3 {
                    x: -1000.0,
                    y: -1000.0,
                    z: -3.0,
                },
                max_point: Vector3 {
                    x: 1000.0,
                    y: 1000.0,
                    z: 3.0,
                },
                voxel_size: Vector3 {
                    x: 3.5,
                    y: 3.5,
                    z: 3.5,
                },
            },
            viz_map: true,
        }
    }
}

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

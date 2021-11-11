#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsComplex32 {
        pub real: f32,
        pub imag: f32,
    }
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsHadmapRoute {
        pub header: StdMsgsHeader,
        pub start_point: AutowareAutoMsgsRoutePoint,
        pub goal_point: AutowareAutoMsgsRoutePoint,
        pub segments: Vec<AutowareAutoMsgsHadmapSegment>,
    }
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsHadmapSegment {
        pub primitives: Vec<AutowareAutoMsgsMapPrimitive>,
        pub preferred_primitive_id: i64,
    }
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsMapPrimitive {
        pub id: i64,
        pub primitive_type: String,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsRoutePoint {
        pub position: GeometryMsgsPoint,
        pub heading: AutowareAutoMsgsComplex32,
    }
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsTrajectory {
        pub header: StdMsgsHeader,
        pub points: Vec<AutowareAutoMsgsTrajectoryPoint>,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsTrajectoryPoint {
        pub time_from_start: BuiltinInterfacesDuration,
        pub x: f32,
        pub y: f32,
        pub heading: AutowareAutoMsgsComplex32,
        pub longitudinal_velocity_mps: f32,
        pub lateral_velocity_mps: f32,
        pub acceleration_mps2: f32,
        pub heading_rate_rps: f32,
        pub front_wheel_angle_rad: f32,
        pub rear_wheel_angle_rad: f32,
    }
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsVehicleKinematicState {
        pub header: StdMsgsHeader,
        pub state: AutowareAutoMsgsTrajectoryPoint,
        pub delta: GeometryMsgsTransform,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsVehicleStateCommand {
        pub stamp: BuiltinInterfacesTime,
        pub blinker: u8,
        pub headlight: u8,
        pub wiper: u8,
        pub gear: u8,
        pub mode: u8,
        pub hand_brake: bool,
        pub horn: bool,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsVehicleStateReport {
        pub stamp: BuiltinInterfacesTime,
        pub fuel: u8,
        pub blinker: u8,
        pub headlight: u8,
        pub wiper: u8,
        pub gear: u8,
        pub mode: u8,
        pub hand_brake: bool,
        pub horn: bool,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct BuiltinInterfacesDuration {
        pub sec: i32,
        pub nanosec: u32,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct BuiltinInterfacesTime {
        pub sec: i32,
        pub nanosec: u32,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsTransform {
        pub translation: GeometryMsgsVector3,
        pub rotation: GeometryMsgsQuaternion,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsPoint {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsPose {
        pub position: GeometryMsgsPoint,
        pub orientation: GeometryMsgsQuaternion,
    }
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsPoseStamped {
        pub header: StdMsgsHeader,
        pub pose: GeometryMsgsPose,
    }
    #[derive(Copy, Clone, Debug, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsPoseWithCovariance {
        pub pose: GeometryMsgsPose,
        pub covariance: [f64; 36],
    }
    #[derive(Clone, Debug, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsPoseWithCovarianceStamped {
        pub header: StdMsgsHeader,
        pub pose: GeometryMsgsPoseWithCovariance,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsQuaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct GeometryMsgsVector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct StdMsgsHeader {
        pub stamp: BuiltinInterfacesTime,
        pub frame_id: String,
    }

    unsafe extern "C++" {
        type InitPose;
        fn init_init_pose() -> UniquePtr<InitPose>;
        fn get_init_pose(node: &mut UniquePtr<InitPose>) -> GeometryMsgsPoseWithCovarianceStamped;
        fn is_new_init_pose(node: &mut UniquePtr<InitPose>) -> bool;
    }

    unsafe extern "C++" {
        type GoalPose;
        fn init_goal_pose() -> UniquePtr<GoalPose>;
        fn get_goal_pose(node: &mut UniquePtr<GoalPose>) -> GeometryMsgsPoseStamped;
        fn is_new_goal_pose(node: &mut UniquePtr<GoalPose>) -> bool;
    }
    pub struct CfgOsmMapLoader {
        pub map_osm_file: String,
        pub origin_offset_lat: f64,
        pub origin_offset_lon: f64,
        pub latitude: f64,
        pub longitude: f64,
        pub elevation: f64,
    }
    unsafe extern "C++" {
        type OsmMapLoader;
        fn init_osm_map_loader(cfg: &CfgOsmMapLoader) -> UniquePtr<OsmMapLoader>;
    }
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    pub struct MapConfig {
        pub capacity: i64,
        pub min_point: Vector3,
        pub max_point: Vector3,
        pub voxel_size: Vector3,
    }
    pub struct CfgPcdMapLoader {
        pub map_pcd_file: String,
        pub map_yaml_file: String,
        pub map_frame: String,
        pub map_config: MapConfig,
        pub viz_map: bool,
    }
    unsafe extern "C++" {
        type PcdMapLoader;
        fn init_pcd_map_loader(cfg: &CfgPcdMapLoader) -> UniquePtr<PcdMapLoader>;
    }
    unsafe extern "C++" {
        type GlobalPlanner;
        fn init_global_planner() -> UniquePtr<GlobalPlanner>;
        fn set_current_pose(
            node: &mut UniquePtr<GlobalPlanner>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn set_goal_pose(node: &mut UniquePtr<GlobalPlanner>, msg: &GeometryMsgsPoseStamped);
        fn get_route(node: &mut UniquePtr<GlobalPlanner>) -> AutowareAutoMsgsHadmapRoute;
    }
    pub struct Vehicle {
        cg_to_front_m: f64,
        cg_to_rear_m: f64,
        front_overhang_m: f64,
        rear_overhang_m: f64,
    }
    pub struct CfgLocalPlanner {
        pub enable_object_collision_estimator: bool,
        pub heading_weight: f64,
        pub goal_distance_thresh: f64,
        pub stop_velocity_thresh: f64,
        pub subroute_goal_offset_lane2parking: f64,
        pub subroute_goal_offset_parking2lane: f64,
        pub vehicle: Vehicle,
    }
    unsafe extern "C++" {
        type LocalPlanner;
        fn init_local_planner(cfg: &CfgLocalPlanner) -> UniquePtr<LocalPlanner>;
        fn set_route(node: &mut UniquePtr<LocalPlanner>, msg: &AutowareAutoMsgsHadmapRoute);
        fn set_kinematic_state(
            node: &mut UniquePtr<LocalPlanner>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn set_state_report(
            node: &mut UniquePtr<LocalPlanner>,
            msg: &AutowareAutoMsgsVehicleStateReport,
        );
        fn get_trajectory(node: &mut UniquePtr<LocalPlanner>) -> AutowareAutoMsgsTrajectory;
        fn get_state_cmd(node: &mut UniquePtr<LocalPlanner>)
            -> AutowareAutoMsgsVehicleStateCommand;
    }
    unsafe extern "C++" {
        type LanePlanner;
    }
    unsafe extern "C++" {
        type ParkingPlanner;
    }
    unsafe extern "C++" {
        type PurePursuit;
    }
    unsafe extern "C++" {
        type Simulator;
    }
}

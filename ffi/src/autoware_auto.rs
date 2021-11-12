#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {

    //Msg types

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
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct AutowareAutoMsgsVehicleControlCommand {
        pub stamp: BuiltinInterfacesTime,
        pub long_accel_mps2: f32,
        pub velocity_mps: f32,
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

    //Config types

    pub struct CfgOsmMapLoader {
        pub map_osm_file: String,
        pub origin_offset_lat: f64,
        pub origin_offset_lon: f64,
        pub latitude: f64,
        pub longitude: f64,
        pub elevation: f64,
    }
    #[derive(Debug)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    #[derive(Debug)]
    pub struct MapConfig {
        pub capacity: i64,
        pub min_point: Vector3,
        pub max_point: Vector3,
        pub voxel_size: Vector3,
    }
    #[derive(Debug)]
    pub struct CfgPcdMapLoader {
        pub map_pcd_file: String,
        pub map_yaml_file: String,
        pub map_frame: String,
        pub map_config: MapConfig,
        pub viz_map: bool,
    }
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
    pub struct CfgLocalPlanner {
        pub enable_object_collision_estimator: bool,
        pub heading_weight: f64,
        pub goal_distance_thresh: f64,
        pub stop_velocity_thresh: f64,
        pub subroute_goal_offset_lane2parking: f64,
        pub subroute_goal_offset_parking2lane: f64,
        pub vehicle: Vehicle,
    }
    pub struct LanePlannerConfig {
        pub trajectory_resolution: f64,
    }
    pub struct GaussianSmoother {
        pub standard_deviation: f64,
        pub kernel_size: i64,
    }
    pub struct CfgLanePlanner {
        pub heading_weight: f64,
        pub lane_planner: LanePlannerConfig,
        pub vehicle: Vehicle,
        pub gaussian_smoother: GaussianSmoother,
    }
    pub struct OptimizationWeights {
        pub steering: f64,
        pub throttle: f64,
        pub goal: f64,
    }
    pub struct StateBound {
        pub x_m: f64,
        pub y_m: f64,
        pub velocity_mps: f64,
        pub heading_rad: f64,
        pub steering_rad: f64,
    }
    pub struct StateBounds {
        pub lower: StateBound,
        pub upper: StateBound,
    }
    pub struct CommandBound {
        pub steering_rate_rps: f64,
        pub throttle_mps2: f64,
    }
    pub struct CommandBounds {
        pub lower: CommandBound,
        pub upper: CommandBound,
    }
    pub struct CfgParkingPlanner {
        pub vehicle: Vehicle,
        pub optimization_weights: OptimizationWeights,
        pub state_bounds: StateBounds,
        pub command_bounds: CommandBounds,
    }
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

    //C++ API

    unsafe extern "C++" {
        type InitPose;
        fn init_pose_init() -> UniquePtr<InitPose>;
        fn init_pose_get_init_pose(
            node: &mut UniquePtr<InitPose>,
        ) -> GeometryMsgsPoseWithCovarianceStamped;
        fn init_pose_is_new_init_pose(node: &mut UniquePtr<InitPose>) -> bool;
    }

    unsafe extern "C++" {
        type GoalPose;
        fn goal_pose_init() -> UniquePtr<GoalPose>;
        fn goal_pose_get_goal_pose(node: &mut UniquePtr<GoalPose>) -> GeometryMsgsPoseStamped;
        fn goal_pose_is_new_goal_pose(node: &mut UniquePtr<GoalPose>) -> bool;
    }
    unsafe extern "C++" {
        type OsmMapLoader;
        fn osm_map_loader_init(cfg: &CfgOsmMapLoader) -> UniquePtr<OsmMapLoader>;
    }
    unsafe extern "C++" {
        type PcdMapLoader;
        fn pcd_map_loader_init(cfg: &CfgPcdMapLoader) -> UniquePtr<PcdMapLoader>;
    }
    unsafe extern "C++" {
        type GlobalPlanner;
        fn global_planner_init() -> UniquePtr<GlobalPlanner>;
        fn global_planner_set_current_pose(
            node: &mut UniquePtr<GlobalPlanner>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn global_planner_set_goal_pose(
            node: &mut UniquePtr<GlobalPlanner>,
            msg: &GeometryMsgsPoseStamped,
        );
        fn global_planner_get_route(
            node: &mut UniquePtr<GlobalPlanner>,
        ) -> AutowareAutoMsgsHadmapRoute;
    }
    unsafe extern "C++" {
        type LocalPlanner;
        fn local_planner_init(cfg: &CfgLocalPlanner) -> UniquePtr<LocalPlanner>;
        fn local_planner_set_route(
            node: &mut UniquePtr<LocalPlanner>,
            msg: &AutowareAutoMsgsHadmapRoute,
        );
        fn local_planner_set_kinematic_state(
            node: &mut UniquePtr<LocalPlanner>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn local_planner_set_state_report(
            node: &mut UniquePtr<LocalPlanner>,
            msg: &AutowareAutoMsgsVehicleStateReport,
        );
        fn local_planner_get_trajectory(
            node: &mut UniquePtr<LocalPlanner>,
        ) -> AutowareAutoMsgsTrajectory;
        fn local_planner_get_state_cmd(
            node: &mut UniquePtr<LocalPlanner>,
        ) -> AutowareAutoMsgsVehicleStateCommand;
    }
    unsafe extern "C++" {
        type LanePlanner;
        fn lane_planner_init(cfg: &CfgLanePlanner) -> UniquePtr<LanePlanner>;
    }
    unsafe extern "C++" {
        type ParkingPlanner;
        fn parking_planner_init(cfg: &CfgParkingPlanner) -> UniquePtr<ParkingPlanner>;
    }
    unsafe extern "C++" {
        type PurePursuit;
        fn pure_pursuit_init(cfg: &CfgPurePursuit) -> UniquePtr<PurePursuit>;
        fn pure_pursuit_set_trajectory(
            node: &mut UniquePtr<PurePursuit>,
            msg: &AutowareAutoMsgsTrajectory,
        );
        fn pure_pursuit_set_kinematic_state(
            node: &mut UniquePtr<PurePursuit>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn pure_pursuit_get_control_cmd(
            node: &mut UniquePtr<PurePursuit>,
        ) -> AutowareAutoMsgsVehicleControlCommand;
    }
    unsafe extern "C++" {
        type Simulator;
        fn simulator_init(cfg: &CfgSimulator) -> UniquePtr<Simulator>;
        fn simulator_set_init_pose(
            node: &mut UniquePtr<Simulator>,
            msg: &GeometryMsgsPoseWithCovarianceStamped,
        );
        fn simulator_set_control_cmd(
            node: &mut UniquePtr<Simulator>,
            msg: &AutowareAutoMsgsVehicleControlCommand,
        );
        fn simulator_set_state_cmd(
            node: &mut UniquePtr<Simulator>,
            msg: &AutowareAutoMsgsVehicleStateCommand,
        );
        fn simulator_get_kinematic_state(
            node: &mut UniquePtr<Simulator>,
        ) -> AutowareAutoMsgsVehicleKinematicState;
        fn simulator_get_state_report(
            node: &mut UniquePtr<Simulator>,
        ) -> AutowareAutoMsgsVehicleStateReport;
        fn simulator_update(node: &mut UniquePtr<Simulator>);
    }
}

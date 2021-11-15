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
        pub z: f32,
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
}

#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    #[derive(Copy, Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct BuiltinInterfacesTime {
        pub sec: i32,
        pub nanosec: u32,
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
    #[derive(Clone, Debug, Default, Serialize, Deserialize, ZFData, ZFFakeSerialize)]
    pub struct StdMsgsHeader {
        pub stamp: BuiltinInterfacesTime,
        pub frame_id: String,
    }

    unsafe extern "C++" {
        include!("zenoh_flow_init_pose/zenoh_flow_init_pose.hpp");
        type InitPose;
        fn init_init_pose() -> UniquePtr<InitPose>;
        fn get_init_pose(node: &mut UniquePtr<InitPose>) -> GeometryMsgsPoseWithCovarianceStamped;
        fn is_new_init_pose(node: &mut UniquePtr<InitPose>) -> bool;
    }

    unsafe extern "C++" {
        include!("zenoh_flow_goal_pose/zenoh_flow_goal_pose.hpp");
        type GoalPose;
        fn init_goal_pose() -> UniquePtr<GoalPose>;
        fn get_goal_pose(node: &mut UniquePtr<GoalPose>) -> GeometryMsgsPoseWithCovarianceStamped;
        fn is_new_goal_pose(node: &mut UniquePtr<GoalPose>) -> bool;
    }
}

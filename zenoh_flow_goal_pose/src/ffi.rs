#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    #[derive(DefaultConfig)]
    struct NativeConfig {
        pub node_name: String,
    }
    unsafe extern "C++" {
        type NativeNode = common::autoware_auto::ffi::NativeNode;
        type GeometryMsgsPoseStamped = common::autoware_auto::msgs::ffi::GeometryMsgsPoseStamped;

        fn init(cfg: &NativeConfig) -> UniquePtr<NativeNode>;
        fn get_goal_pose(node: &mut UniquePtr<NativeNode>) -> GeometryMsgsPoseStamped;
        fn is_new(node: &mut UniquePtr<NativeNode>) -> bool;
    }
}

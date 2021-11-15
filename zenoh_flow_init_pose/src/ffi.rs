#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    #[derive(DefaultConfig)]
    struct NativeConfig {
        pub node_name: String,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_init_pose/zenoh_flow_init_pose.hpp");
        type NativeNode = autoware_auto::ffi::NativeNode;
        type GeometryMsgsPoseWithCovarianceStamped =
            autoware_auto::msgs::ffi::GeometryMsgsPoseWithCovarianceStamped;

        fn init(cfg: &NativeConfig) -> UniquePtr<NativeNode>;
        fn get_init_pose(node: &mut UniquePtr<NativeNode>)
            -> GeometryMsgsPoseWithCovarianceStamped;
        fn is_new(node: &mut UniquePtr<NativeNode>) -> bool;
    }
}

#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    unsafe extern "C++" {
        type NativeNode = common::autoware_auto::ffi::NativeNode;
        type GeometryMsgsPoseWithCovarianceStamped =
            common::autoware_auto::msgs::ffi::GeometryMsgsPoseWithCovarianceStamped;

        fn get_init_pose(node: &mut UniquePtr<NativeNode>)
            -> GeometryMsgsPoseWithCovarianceStamped;
        fn is_new(node: &mut UniquePtr<NativeNode>) -> bool;
    }
}

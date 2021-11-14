#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    #[derive(DefaultConfig)]
    struct NativeConfig {
        pub node_name: String,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_global_planner/zenoh_flow_global_planner.hpp");
        type NativeNode = autoware_auto::ffi::NativeNode;
        type AutowareAutoMsgsVehicleKinematicState =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleKinematicState;
        type GeometryMsgsPoseStamped = autoware_auto::msgs::ffi::GeometryMsgsPoseStamped;
        type AutowareAutoMsgsHadmapRoute = autoware_auto::msgs::ffi::AutowareAutoMsgsHadmapRoute;

        fn init(cfg: &NativeConfig) -> UniquePtr<NativeNode>;
        fn set_current_pose(
            node: &mut UniquePtr<NativeNode>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn set_goal_pose(node: &mut UniquePtr<NativeNode>, msg: &GeometryMsgsPoseStamped);
        fn get_route(node: &mut UniquePtr<NativeNode>) -> AutowareAutoMsgsHadmapRoute;
    }
}

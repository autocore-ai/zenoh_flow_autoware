
use cxx::UniquePtr;
use std::{
    any::type_name,
    fmt::{Debug, Formatter, Result},
};
use zenoh_flow::zenoh_flow_derive::ZFState;

#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    #[derive(DefaultConfig)]
    struct NativeConfig {
        pub node_name: String,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_global_planner/zenoh_flow_global_planner.hpp");
        type NativeNode_global_planner;
        fn init_null_config() -> UniquePtr<NativeNode_global_planner>;
        type AutowareAutoMsgsVehicleKinematicState =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleKinematicState;
        type GeometryMsgsPoseStamped = autoware_auto::msgs::ffi::GeometryMsgsPoseStamped;
        type AutowareAutoMsgsHadmapRoute = autoware_auto::msgs::ffi::AutowareAutoMsgsHadmapRoute;

        fn init_global_planner(cfg: &NativeConfig) -> UniquePtr<NativeNode_global_planner>;
        fn set_current_pose(
            node: &mut UniquePtr<NativeNode_global_planner>,
            msg: &AutowareAutoMsgsVehicleKinematicState,
        );
        fn set_goal_pose(node: &mut UniquePtr<NativeNode_global_planner>, msg: &GeometryMsgsPoseStamped);
        fn get_route(node: &mut UniquePtr<NativeNode_global_planner>) -> AutowareAutoMsgsHadmapRoute;
    }
}

unsafe impl Send for ffi::NativeNode_global_planner {}
unsafe impl Sync for ffi::NativeNode_global_planner {}

impl Debug for ffi::NativeNode_global_planner {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_global_planner>()).finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_global_planner>,
}

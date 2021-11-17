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
        include!("zenoh_flow_goal_pose/zenoh_flow_goal_pose.hpp");
        type NativeNode_goal_pose;

        type GeometryMsgsPoseStamped = autoware_auto::msgs::ffi::GeometryMsgsPoseStamped;

        fn init_goal_pose(cfg: &NativeConfig) -> UniquePtr<NativeNode_goal_pose>;
        fn get_goal_pose(node: &mut UniquePtr<NativeNode_goal_pose>) -> GeometryMsgsPoseStamped;
        fn is_new_goal_pose(node: &mut UniquePtr<NativeNode_goal_pose>) -> bool;
    }
}
unsafe impl Send for ffi::NativeNode_goal_pose {}
unsafe impl Sync for ffi::NativeNode_goal_pose {}

impl Debug for ffi::NativeNode_goal_pose {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_goal_pose>())
            .finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_goal_pose>,
}

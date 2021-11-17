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
        include!("zenoh_flow_init_pose/zenoh_flow_init_pose.hpp");
        type NativeNode_init_pose;
        type GeometryMsgsPoseWithCovarianceStamped =
            autoware_auto::msgs::ffi::GeometryMsgsPoseWithCovarianceStamped;

        fn init_init_pose(cfg: &NativeConfig) -> UniquePtr<NativeNode_init_pose>;
        fn get_init_pose(
            node: &mut UniquePtr<NativeNode_init_pose>,
        ) -> GeometryMsgsPoseWithCovarianceStamped;
        fn is_new_init_pose(node: &mut UniquePtr<NativeNode_init_pose>) -> bool;
    }
}

unsafe impl Send for ffi::NativeNode_init_pose {}
unsafe impl Sync for ffi::NativeNode_init_pose {}

impl Debug for ffi::NativeNode_init_pose {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_init_pose>())
            .finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_init_pose>,
}

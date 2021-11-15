pub mod default_msg;
pub mod msgs;

use cxx::UniquePtr;
use std::{
    any::type_name,
    fmt::{Debug, Formatter, Result},
};
use zenoh_flow::zenoh_flow_derive::ZFState;

#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    unsafe extern "C++" {
        type NativeNode;
        fn init_null_config() -> UniquePtr<NativeNode>;
    }
}

unsafe impl Send for ffi::NativeNode {}
unsafe impl Sync for ffi::NativeNode {}

impl Debug for ffi::NativeNode {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode>()).finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode>,
}

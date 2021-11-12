use crate::autoware_auto::ffi::*;
use std::{
    any::type_name,
    fmt::{Debug, Formatter, Result},
};

unsafe impl Send for InitPose {}
unsafe impl Send for PcdMapLoader {}
unsafe impl Sync for InitPose {}
unsafe impl Sync for PcdMapLoader {}

fn debug_fmt<T>(_: &T, f: &mut Formatter<'_>) -> Result {
    f.debug_struct(type_name::<T>()).finish()
}

impl Debug for InitPose {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        debug_fmt::<InitPose>(self, f)
    }
}
impl Debug for PcdMapLoader {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        debug_fmt::<PcdMapLoader>(self, f)
    }
}

use crate::autoware_auto::ffi::*;
use std::{
    any::type_name,
    fmt::{Debug, Formatter, Result},
};

unsafe impl Send for InitPose {}
unsafe impl Send for OsmMapLoader {}
unsafe impl Send for PcdMapLoader {}
unsafe impl Sync for InitPose {}
unsafe impl Sync for OsmMapLoader {}
unsafe impl Sync for PcdMapLoader {}

fn fmt<T>(_: &T, f: &mut Formatter<'_>) -> Result {
    f.debug_struct(type_name::<T>()).finish()
}

impl Debug for InitPose {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for OsmMapLoader {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for PcdMapLoader {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}

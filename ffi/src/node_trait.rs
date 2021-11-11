use crate::autoware_auto::ffi::*;
use std::fmt::{Debug, Formatter, Result};

unsafe impl Send for InitPose {}
unsafe impl Sync for InitPose {}
impl Debug for InitPose {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct("native InitPose node").finish()
    }
}

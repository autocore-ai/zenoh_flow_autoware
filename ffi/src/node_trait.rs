use crate::autoware_auto::ffi::*;
use std::{
    any::type_name,
    fmt::{Debug, Formatter, Result},
};

unsafe impl Send for GlobalPlanner {}
unsafe impl Send for GoalPose {}
unsafe impl Send for InitPose {}
unsafe impl Send for LanePlanner {}
unsafe impl Send for LocalPlanner {}
unsafe impl Send for OsmMapLoader {}
unsafe impl Send for ParkingPlanner {}
unsafe impl Send for PcdMapLoader {}
unsafe impl Send for PurePursuit {}
unsafe impl Send for Simulator {}
unsafe impl Sync for GlobalPlanner {}
unsafe impl Sync for GoalPose {}
unsafe impl Sync for InitPose {}
unsafe impl Sync for LanePlanner {}
unsafe impl Sync for LocalPlanner {}
unsafe impl Sync for OsmMapLoader {}
unsafe impl Sync for ParkingPlanner {}
unsafe impl Sync for PcdMapLoader {}
unsafe impl Sync for PurePursuit {}
unsafe impl Sync for Simulator {}

fn fmt<T>(_: &T, f: &mut Formatter<'_>) -> Result {
    f.debug_struct(type_name::<T>()).finish()
}

impl Debug for GlobalPlanner {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for GoalPose {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for InitPose {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for LanePlanner {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for LocalPlanner {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for OsmMapLoader {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for ParkingPlanner {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for PcdMapLoader {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for PurePursuit {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}
impl Debug for Simulator {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        fmt(self, f)
    }
}

pub mod configs;
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
    // unsafe extern "C++" {
    //     type Simulator;
    //     fn simulator_init(cfg: &CfgSimulator) -> UniquePtr<Simulator>;
    //     fn simulator_set_init_pose(
    //         node: &mut UniquePtr<Simulator>,
    //         msg: &GeometryMsgsPoseWithCovarianceStamped,
    //     );
    //     fn simulator_set_control_cmd(
    //         node: &mut UniquePtr<Simulator>,
    //         msg: &AutowareAutoMsgsVehicleControlCommand,
    //     );
    //     fn simulator_set_state_cmd(
    //         node: &mut UniquePtr<Simulator>,
    //         msg: &AutowareAutoMsgsVehicleStateCommand,
    //     );
    //     fn simulator_get_kinematic_state(
    //         node: &mut UniquePtr<Simulator>,
    //     ) -> AutowareAutoMsgsVehicleKinematicState;
    //     fn simulator_get_state_report(
    //         node: &mut UniquePtr<Simulator>,
    //     ) -> AutowareAutoMsgsVehicleStateReport;
    //     fn simulator_update(node: &mut UniquePtr<Simulator>);
    // }
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

// Copyright 2021 The AutoCore.AI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use cxx::UniquePtr;
use std::{
    any::type_name,
    fmt::{Debug, Formatter, Result},
};
use zenoh_flow::zenoh_flow_derive::ZFState;

#[cxx::bridge(namespace = "zenoh_flow::autoware_auto::ffi")]
pub mod ffi {
    struct NativeConfig {
        pub simulated_frame_id: String,
        pub origin_frame_id: String,
        pub vehicle_model_type: String,
        pub initialize_source: String,
        pub timer_sampling_time_ms: i64,
        pub add_measurement_noise: bool,
        pub vel_lim: f64,
        pub vel_rate_lim: f64,
        pub steer_lim: f64,
        pub steer_rate_lim: f64,
        pub acc_time_delay: f64,
        pub acc_time_constant: f64,
        pub steer_time_delay: f64,
        pub steer_time_constant: f64,
    }
    unsafe extern "C++" {
        include!("zenoh_flow_simulator/zenoh_flow_simulator.hpp");
        type NativeNode_simulator;
        type GeometryMsgsPoseWithCovarianceStamped =
            autoware_auto::msgs::ffi::GeometryMsgsPoseWithCovarianceStamped;
        type AutowareAutoMsgsVehicleControlCommand =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleControlCommand;
        type AutowareAutoMsgsVehicleStateCommand =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleStateCommand;
        type AutowareAutoMsgsVehicleKinematicState =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleKinematicState;
        type AutowareAutoMsgsVehicleStateReport =
            autoware_auto::msgs::ffi::AutowareAutoMsgsVehicleStateReport;

        fn init_simulator(cfg: &NativeConfig) -> UniquePtr<NativeNode_simulator>;
        fn set_init_pose(
            node: &mut UniquePtr<NativeNode_simulator>,
            msg: &GeometryMsgsPoseWithCovarianceStamped,
        );
        fn set_control_cmd(
            node: &mut UniquePtr<NativeNode_simulator>,
            msg: &AutowareAutoMsgsVehicleControlCommand,
        );
        fn set_state_cmd(
            node: &mut UniquePtr<NativeNode_simulator>,
            msg: &AutowareAutoMsgsVehicleStateCommand,
        );
        fn get_kinematic_state(
            node: &mut UniquePtr<NativeNode_simulator>,
        ) -> AutowareAutoMsgsVehicleKinematicState;
        fn get_state_report(
            node: &mut UniquePtr<NativeNode_simulator>,
        ) -> AutowareAutoMsgsVehicleStateReport;
        fn update(node: &mut UniquePtr<NativeNode_simulator>);
        fn is_initialized(node: &mut UniquePtr<NativeNode_simulator>) -> bool;
    }
}

unsafe impl Send for ffi::NativeNode_simulator {}
unsafe impl Sync for ffi::NativeNode_simulator {}

impl Debug for ffi::NativeNode_simulator {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.debug_struct(type_name::<ffi::NativeNode_simulator>())
            .finish()
    }
}

#[derive(Debug, ZFState)]
pub struct NativeNodeInstance {
    pub ptr: UniquePtr<ffi::NativeNode_simulator>,
}

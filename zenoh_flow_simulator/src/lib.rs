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

mod ffi;

use autoware_auto::msgs::ffi::{
    AutowareAutoMsgsVehicleControlCommand, AutowareAutoMsgsVehicleStateCommand,
    GeometryMsgsPoseWithCovarianceStamped,
};
use derive::{zf_default_node, DefaultSendAndSync};
use ffi::ffi::{
    get_kinematic_state, get_state_report, init_simulator, is_initialized, set_control_cmd,
    set_init_pose, set_state_cmd, update, NativeConfig,
};
use ffi::NativeNodeInstance;
use std::{collections::HashMap, fmt::Debug, sync::Arc};
use zenoh_flow::{
    default_output_rule, export_operator, runtime::message::DataMessage,
    zenoh_flow_derive::ZFState, Configuration, Context, Data, DeadlineMiss, Node, NodeOutput,
    Operator, PortId, State, Token, ZFError, ZFResult,
};

const IN_TICK: &str = "tick";
const IN_INIT_POSE: &str = "init_pose";
const IN_CONTROL_CMD: &str = "control_cmd";
const IN_STATE_CMD: &str = "state_cmd";
const OUT_KINEMATIC_STATE: &str = "kinematic_state";
const OUT_STATE_REPORT: &str = "state_report";

const TICK_MODE: usize = 1;
const INIT_POSE_MODE: usize = 2;
const CONTROL_CMD_MODE: usize = 3;
const STATE_CMD_MODE: usize = 4;

#[zf_default_node(init_fn = "init_simulator")]
#[derive(Debug, ZFState, DefaultSendAndSync)]
pub struct CustomNode;

impl Default for NativeConfig {
    fn default() -> Self {
        NativeConfig {
            simulated_frame_id: String::from("base_link"),
            origin_frame_id: String::from("map"),
            vehicle_model_type: String::from("IDEAL_STEER_VEL"),
            initialize_source: String::from("INITIAL_POSE_TOPIC"),
            timer_sampling_time_ms: 25,
            add_measurement_noise: false,
            vel_lim: 30.0,
            vel_rate_lim: 30.0,
            steer_lim: 0.6,
            steer_rate_lim: 6.28,
            acc_time_delay: 0.1,
            acc_time_constant: 0.1,
            steer_time_delay: 0.1,
            steer_time_constant: 0.1,
        }
    }
}

fn get_config(configuration: &Option<Configuration>) -> NativeConfig {
    match configuration {
        Some(config) => {
            let simulated_frame_id = match config["simulated_frame_id"].as_str() {
                Some(v) => String::from(v),
                None => NativeConfig::default().simulated_frame_id,
            };
            let origin_frame_id = match config["origin_frame_id"].as_str() {
                Some(v) => String::from(v),
                None => NativeConfig::default().origin_frame_id,
            };
            let vehicle_model_type = match config["vehicle_model_type"].as_str() {
                Some(v) => String::from(v),
                None => NativeConfig::default().vehicle_model_type,
            };
            let initialize_source = match config["initialize_source"].as_str() {
                Some(v) => String::from(v),
                None => NativeConfig::default().initialize_source,
            };
            let timer_sampling_time_ms = match config["timer_sampling_time_ms"].as_i64() {
                Some(v) => v,
                None => NativeConfig::default().timer_sampling_time_ms,
            };
            let add_measurement_noise = match config["add_measurement_noise"].as_bool() {
                Some(v) => v,
                None => NativeConfig::default().add_measurement_noise,
            };
            let vel_lim = match config["vel_lim"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vel_lim,
            };
            let vel_rate_lim = match config["vel_rate_lim"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vel_rate_lim,
            };
            let steer_lim = match config["steer_lim"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().steer_lim,
            };
            let steer_rate_lim = match config["steer_rate_lim"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().steer_rate_lim,
            };
            let acc_time_delay = match config["acc_time_delay"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().acc_time_delay,
            };
            let acc_time_constant = match config["acc_time_constant"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().acc_time_constant,
            };
            let steer_time_delay = match config["steer_time_delay"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().steer_time_delay,
            };
            let steer_time_constant = match config["steer_time_constant"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().steer_time_constant,
            };
            NativeConfig {
                simulated_frame_id,
                origin_frame_id,
                vehicle_model_type,
                initialize_source,
                timer_sampling_time_ms,
                add_measurement_noise,
                vel_lim,
                vel_rate_lim,
                steer_lim,
                steer_rate_lim,
                acc_time_delay,
                acc_time_constant,
                steer_time_delay,
                steer_time_constant,
            }
        }
        None => NativeConfig::default(),
    }
}

impl Operator for CustomNode {
    fn input_rule(
        &self,
        context: &mut Context,
        _state: &mut State,
        tokens: &mut HashMap<zenoh_flow::PortId, Token>,
    ) -> ZFResult<bool> {
        for (port_id, token) in tokens.into_iter() {
            match token {
                Token::Ready(_) => {
                    match port_id.as_ref() {
                        IN_TICK => context.mode = TICK_MODE,
                        IN_INIT_POSE => context.mode = INIT_POSE_MODE,
                        IN_CONTROL_CMD => context.mode = CONTROL_CMD_MODE,
                        IN_STATE_CMD => context.mode = STATE_CMD_MODE,
                        _ => context.mode = 0,
                    }
                    return Ok(true);
                }
                Token::Pending => continue,
            }
        }

        Ok(false)
    }

    fn run(
        &self,
        context: &mut Context,
        dyn_state: &mut State,
        inputs: &mut HashMap<zenoh_flow::PortId, DataMessage>,
    ) -> ZFResult<HashMap<zenoh_flow::PortId, Data>> {
        let mut results = HashMap::with_capacity(2);

        let node = &mut dyn_state.try_get::<NativeNodeInstance>()?.ptr;

        match context.mode {
            TICK_MODE => {
                if is_initialized(node) {
                    update(node);
                    results.insert(
                        OUT_KINEMATIC_STATE.into(),
                        Data::from(get_kinematic_state(node)),
                    );
                    results.insert(OUT_STATE_REPORT.into(), Data::from(get_state_report(node)));
                } else {
                    println!("Simulator waiting initialization...")
                }
            }
            INIT_POSE_MODE => {
                let mut data_msgs = inputs
                    .remove(IN_INIT_POSE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let msg = data_msgs
                    .data
                    .try_get::<GeometryMsgsPoseWithCovarianceStamped>()?;
                set_init_pose(node, msg);
            }
            CONTROL_CMD_MODE => {
                let mut data_msgs = inputs
                    .remove(IN_CONTROL_CMD)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let msg = data_msgs
                    .data
                    .try_get::<AutowareAutoMsgsVehicleControlCommand>()?;
                set_control_cmd(node, msg);
            }
            STATE_CMD_MODE => {
                let mut data_msgs = inputs
                    .remove(IN_STATE_CMD)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let msg = data_msgs
                    .data
                    .try_get::<AutowareAutoMsgsVehicleStateCommand>()?;
                set_state_cmd(node, msg);
            }
            _ => {
                log::error!("Simulator, unknown context mode: {}", context.mode);
            }
        };
        Ok(results)
    }

    fn output_rule(
        &self,
        _context: &mut Context,
        state: &mut State,
        outputs: HashMap<PortId, Data>,
        _deadline_miss: Option<DeadlineMiss>,
    ) -> ZFResult<HashMap<PortId, NodeOutput>> {
        default_output_rule(state, outputs)
    }
}

export_operator!(register);

fn register() -> ZFResult<Arc<dyn Operator>> {
    Ok(Arc::new(CustomNode) as Arc<dyn Operator>)
}

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

use autoware_auto::msgs::ffi::{AutowareAutoMsgsTrajectory, AutowareAutoMsgsVehicleKinematicState};
use derive::{zf_default_node, DefaultSendAndSync};
use ffi::ffi::{
    get_control_cmd, init_pure_pursuit, set_kinematic_state, set_trajectory, NativeConfig,
};
use ffi::NativeNodeInstance;
use std::{collections::HashMap, fmt::Debug, sync::Arc};
use zenoh_flow::{
    default_output_rule, export_operator, runtime::message::DataMessage,
    zenoh_flow_derive::ZFState, Configuration, Context, Data, LocalDeadlineMiss, Node, NodeOutput,
    Operator, PortId, State, Token, ZFError, ZFResult,
};

const IN_KINEMATIC_STATE: &str = "kinematic_state";
const IN_TRAJECTORY: &str = "trajectory";
const OUT_CONTROL_COMMAND: &str = "control_cmd";

const KINEMATIC_STATE_MODE: usize = 1;
const TRAJECTORY_MODE: usize = 2;

#[zf_default_node(init_fn = "init_pure_pursuit")]
#[derive(Debug, ZFState, DefaultSendAndSync)]
pub struct CustomNode;

impl Default for NativeConfig {
    fn default() -> Self {
        NativeConfig {
            minimum_lookahead_distance: 6.0,
            maximum_lookahead_distance: 100.0,
            speed_to_lookahead_ratio: 2.0,
            is_interpolate_lookahead_point: true,
            is_delay_compensation: false,
            emergency_stop_distance: 0.1,
            speed_thres_traveling_direction: 0.3,
            dist_front_rear_wheels: 2.7,
        }
    }
}

fn get_config(configuration: &Option<Configuration>) -> NativeConfig {
    match configuration {
        Some(config) => {
            let minimum_lookahead_distance = match config["minimum_lookahead_distance"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().minimum_lookahead_distance,
            };
            let maximum_lookahead_distance = match config["maximum_lookahead_distance"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().maximum_lookahead_distance,
            };
            let speed_to_lookahead_ratio = match config["speed_to_lookahead_ratio"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().speed_to_lookahead_ratio,
            };
            let is_interpolate_lookahead_point =
                match config["is_interpolate_lookahead_point"].as_bool() {
                    Some(v) => v,
                    None => NativeConfig::default().is_interpolate_lookahead_point,
                };
            let is_delay_compensation = match config["is_delay_compensation"].as_bool() {
                Some(v) => v,
                None => NativeConfig::default().is_delay_compensation,
            };
            let emergency_stop_distance = match config["emergency_stop_distance"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().emergency_stop_distance,
            };
            let speed_thres_traveling_direction =
                match config["speed_thres_traveling_direction"].as_f64() {
                    Some(v) => v,
                    None => NativeConfig::default().speed_thres_traveling_direction,
                };
            let dist_front_rear_wheels = match config["dist_front_rear_wheels"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().dist_front_rear_wheels,
            };
            NativeConfig {
                minimum_lookahead_distance,
                maximum_lookahead_distance,
                speed_to_lookahead_ratio,
                is_interpolate_lookahead_point,
                is_delay_compensation,
                emergency_stop_distance,
                speed_thres_traveling_direction,
                dist_front_rear_wheels,
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
                        IN_KINEMATIC_STATE => context.mode = KINEMATIC_STATE_MODE,
                        IN_TRAJECTORY => context.mode = TRAJECTORY_MODE,
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
            KINEMATIC_STATE_MODE => {
                let mut data_msg = inputs
                    .remove(IN_KINEMATIC_STATE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let msg = data_msg
                    .get_inner_data()
                    .try_get::<AutowareAutoMsgsVehicleKinematicState>()?;
                set_kinematic_state(node, &msg);
                results.insert(
                    OUT_CONTROL_COMMAND.into(),
                    Data::from(get_control_cmd(node)),
                );
            }
            TRAJECTORY_MODE => {
                let mut data_msgs = inputs
                    .remove(IN_TRAJECTORY)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let msg = data_msgs
                    .get_inner_data()
                    .try_get::<AutowareAutoMsgsTrajectory>()?;
                set_trajectory(node, &msg);
            }
            _ => {
                log::error!("Pure Pursuit, unknown context mode: {}", context.mode);
            }
        };
        Ok(results)
    }

    fn output_rule(
        &self,
        _context: &mut Context,
        state: &mut State,
        outputs: HashMap<PortId, Data>,
        _deadline_miss: Option<LocalDeadlineMiss>,
    ) -> ZFResult<HashMap<PortId, NodeOutput>> {
        default_output_rule(state, outputs)
    }
}

export_operator!(register);

fn register() -> ZFResult<Arc<dyn Operator>> {
    Ok(Arc::new(CustomNode) as Arc<dyn Operator>)
}

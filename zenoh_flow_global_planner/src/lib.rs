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

use autoware_auto::msgs::ffi::{AutowareAutoMsgsVehicleKinematicState, GeometryMsgsPoseStamped};
use derive::{zf_default_node, DefaultSendAndSync};
use ffi::ffi::{get_route, init_global_planner, set_current_pose, set_goal_pose, NativeConfig};
use ffi::NativeNodeInstance;
use std::{collections::HashMap, fmt::Debug, sync::Arc};
use zenoh_flow::{
    default_output_rule, export_operator, runtime::message::DataMessage,
    zenoh_flow_derive::ZFState, Configuration, Context, Data, LocalDeadlineMiss, Node, NodeOutput,
    Operator, State, Token, ZFError, ZFResult,
};

static IN_GOAL_POSE: &str = "goal_pose";
static IN_CURRENT_POSE: &str = "current_pose";
static OUT_ROUTE: &str = "route";

const GOAL_POSE_MODE: usize = 1;
const CURRENT_POSE_MODE: usize = 2;

#[zf_default_node(init_fn = "init_global_planner")]
#[derive(Debug, ZFState, DefaultSendAndSync)]
pub struct CustomNode;

fn get_config(configuration: &Option<Configuration>) -> NativeConfig {
    match configuration {
        Some(config) => {
            let node_name = match config["node_name"].as_str() {
                Some(v) => String::from(v),
                None => NativeConfig::default().node_name,
            };
            NativeConfig { node_name }
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
                    if port_id.as_ref() == IN_GOAL_POSE {
                        context.mode = GOAL_POSE_MODE;
                    } else {
                        context.mode = CURRENT_POSE_MODE;
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
        let node = &mut dyn_state.try_get::<NativeNodeInstance>()?.ptr;
        let mut result: HashMap<zenoh_flow::PortId, Data> = HashMap::with_capacity(1);

        match context.mode {
            GOAL_POSE_MODE => {
                let mut data_msg = inputs
                    .remove(IN_GOAL_POSE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let msg = data_msg
                    .get_inner_data()
                    .try_get::<GeometryMsgsPoseStamped>()?;

                set_goal_pose(node, &msg);

                let hadmap_route = get_route(node);
                result.insert(OUT_ROUTE.into(), Data::from(hadmap_route));
            }
            CURRENT_POSE_MODE => {
                let mut vehicle_kinematic_state_data_message = inputs
                    .remove(IN_CURRENT_POSE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let vehicle_kinematic_state = vehicle_kinematic_state_data_message
                    .get_inner_data()
                    .try_get::<AutowareAutoMsgsVehicleKinematicState>()?;
                set_current_pose(node, &vehicle_kinematic_state);
            }
            _ => {
                log::error!(
                    "Global planner nodes, unknown context mode: {:?}",
                    context.mode
                );
            }
        }

        Ok(result)
    }

    fn output_rule(
        &self,
        _context: &mut Context,
        state: &mut State,
        outputs: HashMap<zenoh_flow::PortId, Data>,
        _deadline_miss: Option<LocalDeadlineMiss>,
    ) -> ZFResult<HashMap<zenoh_flow::PortId, NodeOutput>> {
        default_output_rule(state, outputs)
    }
}

export_operator!(register);

fn register() -> ZFResult<Arc<dyn Operator>> {
    Ok(Arc::new(CustomNode) as Arc<dyn Operator>)
}

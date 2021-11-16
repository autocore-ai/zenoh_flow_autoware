mod ffi;

use autoware_auto::msgs::ffi::{AutowareAutoMsgsVehicleKinematicState, GeometryMsgsPoseStamped};
use ffi::ffi::{get_route, init_global_planner, set_current_pose, set_goal_pose, NativeConfig};
use std::{collections::HashMap, fmt::Debug, sync::Arc};
use zenoh_flow::{
    default_output_rule, export_operator, runtime::message::DataMessage,
    zenoh_flow_derive::ZFState, Configuration, Context, Data, DeadlineMiss, Node, NodeOutput,
    Operator, State, Token, ZFError, ZFResult,
};
use ffi::NativeNodeInstance;

static IN_GOAL_POSE: &str = "goal_pose";
static IN_CURRENT_POSE: &str = "current_pose";
static OUT_ROUTE: &str = "route";

const GOAL_POSE_MODE: usize = 1;
const CURRENT_POSE_MODE: usize = 2;

#[derive(Debug, ZFState)]
pub struct CustomNode;

unsafe impl Send for CustomNode {}
unsafe impl Sync for CustomNode {}

impl Node for CustomNode {
    fn initialize(&self, cfg: &Option<Configuration>) -> ZFResult<State> {
        Ok(State::from(NativeNodeInstance {
            ptr: init_global_planner(&get_config(cfg)),
        }))
    }
    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}

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
                let msg = data_msg.data.try_get::<GeometryMsgsPoseStamped>()?;

                set_goal_pose(node, &msg);

                let hadmap_route = get_route(node);
                result.insert(OUT_ROUTE.into(), Data::from(hadmap_route));
            }
            CURRENT_POSE_MODE => {
                let mut vehicle_kinematic_state_data_message = inputs
                    .remove(IN_CURRENT_POSE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let vehicle_kinematic_state = vehicle_kinematic_state_data_message
                    .data
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
        _deadline_miss: Option<DeadlineMiss>,
    ) -> ZFResult<HashMap<zenoh_flow::PortId, NodeOutput>> {
        default_output_rule(state, outputs)
    }
}

export_operator!(register);

fn register() -> ZFResult<Arc<dyn Operator>> {
    Ok(Arc::new(CustomNode) as Arc<dyn Operator>)
}

use std::{collections::HashMap, fmt::Debug, sync::Arc};
use cxx::UniquePtr;
use ffi::autoware_auto::ffi::{
    global_planner_init, global_planner_set_current_pose, global_planner_set_goal_pose,
    global_planner_get_route, GlobalPlanner, AutowareAutoMsgsVehicleKinematicState,
    GeometryMsgsPoseStamped
};
use zenoh_flow::runtime::message::DataMessage;
use zenoh_flow::{
    default_output_rule, export_operator, Configuration, Context, Data, DeadlineMiss,
    Node, NodeOutput, Operator, State, Token, ZFError,
    ZFResult, zenoh_flow_derive::ZFState,
};

static IN_GOAL_POSE: &str = "goal_pose";
static IN_CURRENT_POSE: &str = "current_pose";
static OUT_ROUTE: &str = "route";

const GOAL_POSE_MODE: usize = 1;
const CURRENT_POSE_MODE: usize = 2;

#[derive(Debug, ZFState)]
pub struct GlobalPlannerOperator;
unsafe impl Send for GlobalPlannerOperator {}
unsafe impl Sync for GlobalPlannerOperator {}

#[derive(Debug, ZFState)]
pub struct Instance {
    pub ptr: UniquePtr<GlobalPlanner>,
}

impl Node for GlobalPlannerOperator {
    fn initialize(&self, _configuration: &Option<Configuration>) -> ZFResult<State> {
        let ptr = global_planner_init();
        Ok(State::from(Instance { ptr }))
    }

    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}

impl Operator for GlobalPlannerOperator {
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
        let instance = dyn_state.try_get::<Instance>()?;
        let ptr = &mut instance.ptr;
        let mut result: HashMap<zenoh_flow::PortId, Data> = HashMap::with_capacity(1);

        match context.mode {
            GOAL_POSE_MODE => {
                let mut goal_pose_message = inputs
                    .remove(IN_GOAL_POSE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let goal_pose = goal_pose_message
                    .data
                    .try_get::<GeometryMsgsPoseStamped>()?;

                global_planner_set_goal_pose(ptr, &goal_pose);

                let hadmap_route = global_planner_get_route(ptr);
                result.insert(OUT_ROUTE.into(), Data::from(hadmap_route));
            }
            CURRENT_POSE_MODE => {
                let mut vehicle_kinematic_state_data_message = inputs
                    .remove(IN_CURRENT_POSE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let vehicle_kinematic_state = vehicle_kinematic_state_data_message
                    .data
                    .try_get::<AutowareAutoMsgsVehicleKinematicState>(
                )?;
                global_planner_set_current_pose(ptr, &vehicle_kinematic_state);
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
    env_logger::init();
    Ok(Arc::new(GlobalPlannerOperator) as Arc<dyn Operator>)
}

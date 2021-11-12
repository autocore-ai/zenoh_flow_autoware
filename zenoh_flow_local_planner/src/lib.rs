use std::{collections::HashMap, fmt::Debug, sync::Arc};
use cxx::UniquePtr;
use ffi::autoware_auto::ffi::{
    local_planner_init, local_planner_set_route, local_planner_set_kinematic_state, local_planner_set_state_report,
    local_planner_get_trajectory, local_planner_get_state_cmd, LocalPlanner, AutowareAutoMsgsHadmapRoute, 
    AutowareAutoMsgsVehicleKinematicState, AutowareAutoMsgsVehicleStateReport, CfgLocalPlanner
};
use zenoh_flow::{
    default_output_rule, export_operator, Configuration, Context, Data, DeadlineMiss,
    Node, NodeOutput, Operator, State, Token, ZFError,
    ZFResult, zenoh_flow_derive::ZFState,
    runtime::message::DataMessage
};

static IN_VEHICLE_KINEMATIC_STATE: &str = "vehicle_kinematic_state";
static IN_HADMAP_ROUTE: &str = "hadmap_route";
static IN_VEHICLE_STATE_REPORT: &str = "vehicle_state_report";
static OUT_TRAJECTORY: &str = "trajectory";
static OUT_VEHICLE_STATE_COMMAND: &str = "vehicle_state_command";

const VEHICLE_KINEMATIC_STATE_MODE: usize = 1;
const HADMAP_ROUTE_MODE: usize = 2;
const VEHICLE_STATE_REPORT_MODE: usize = 3;

#[derive(Debug, ZFState)]
pub struct LocalPlannerOperator;
unsafe impl Send for LocalPlannerOperator {}
unsafe impl Sync for LocalPlannerOperator {}

#[derive(Debug, ZFState)]
pub struct Instance {
    pub ptr: UniquePtr<LocalPlanner>,
}

impl Node for LocalPlannerOperator {
    fn initialize(&self, configuration: &Option<Configuration>) -> ZFResult<State> {
        let cfg = match configuration {
            Some(config) => {
                if let (
                    Some(enable_object_collision_estimator),
                    Some(heading_weight),
                    Some(goal_distance_thresh),
                    Some(stop_velocity_thresh),
                    Some(subroute_goal_offset_lane2parking),
                    Some(subroute_goal_offset_parking2lane),
                    Some(cg_to_front_m),
                    Some(cg_to_rear_m),
                    Some(front_overhang_m),
                    Some(rear_overhang_m),
                ) = (
                    config.get("enable_object_collision_estimator"),
                    config.get("heading_weight"),
                    config.get("goal_distance_thresh"),
                    config.get("stop_velocity_thresh"),
                    config.get("subroute_goal_offset_lane2parking"),
                    config.get("subroute_goal_offset_parking2lane"),
                    config.get("cg_to_front_m"),
                    config.get("cg_to_rear_m"),
                    config.get("front_overhang_m"),
                    config.get("rear_overhang_m"),
                ) {
                    let mut cxx_cfg = CfgLocalPlanner::default();
                    cxx_cfg.enable_object_collision_estimator =
                        enable_object_collision_estimator.as_bool().unwrap();
                    cxx_cfg.heading_weight = heading_weight.as_f64().unwrap();
                    cxx_cfg.goal_distance_thresh = goal_distance_thresh.as_f64().unwrap();
                    cxx_cfg.stop_velocity_thresh = stop_velocity_thresh.as_f64().unwrap();
                    cxx_cfg.subroute_goal_offset_lane2parking =
                        subroute_goal_offset_lane2parking.as_f64().unwrap();
                    cxx_cfg.subroute_goal_offset_parking2lane =
                        subroute_goal_offset_parking2lane.as_f64().unwrap();
                    cxx_cfg.vehicle.cg_to_front_m = cg_to_front_m.as_f64().unwrap();
                    cxx_cfg.vehicle.cg_to_rear_m = cg_to_rear_m.as_f64().unwrap();
                    cxx_cfg.vehicle.front_overhang_m = front_overhang_m.as_f64().unwrap();
                    cxx_cfg.vehicle.rear_overhang_m = rear_overhang_m.as_f64().unwrap();
                    log::info!("Local Planner configuration: {:?}", cxx_cfg);
                    Ok(cxx_cfg)
                } else {
                    Err(ZFError::MissingConfiguration)
                }
            }
            None => Err(ZFError::InvalidData(String::from("Configuration of Local Planner is None")))
        }?;
        
        let ptr = local_planner_init(&cfg);
        Ok(State::from(Instance { ptr }))
    }

    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}
impl Operator for LocalPlannerOperator {
    fn input_rule(
        &self,
        context: &mut Context,
        _state: &mut State,
        tokens: &mut HashMap<zenoh_flow::PortId, Token>,
    ) -> ZFResult<bool> {
        for (port_id, token) in tokens.into_iter() {
            match token {
                Token::Ready(_) => {
                    if port_id.as_ref() == IN_VEHICLE_KINEMATIC_STATE {
                        context.mode = VEHICLE_KINEMATIC_STATE_MODE;
                    } else if port_id.as_ref() == IN_HADMAP_ROUTE {
                        context.mode = HADMAP_ROUTE_MODE;
                    } else {
                        context.mode = VEHICLE_STATE_REPORT_MODE;
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

        log::debug!("Local Planner receive messages ...");
        let instance = dyn_state.try_get::<Instance>()?;
        let ptr = &mut instance.ptr;
        
        match context.mode {
            VEHICLE_KINEMATIC_STATE_MODE => {
                let mut vehicle_kinematic_state_data_message = inputs
                    .remove(IN_VEHICLE_KINEMATIC_STATE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let vehicle_kinematic_state = vehicle_kinematic_state_data_message
                    .data
                    .try_get::<AutowareAutoMsgsVehicleKinematicState>(
                )?;
                local_planner_set_kinematic_state(ptr, &vehicle_kinematic_state);

                let trajectory = local_planner_get_trajectory(ptr);
                let vehicle_state_command = local_planner_get_state_cmd(ptr);
                results.insert(OUT_TRAJECTORY.into(), Data::from(trajectory));
                results.insert(
                    OUT_VEHICLE_STATE_COMMAND.into(),
                    Data::from(vehicle_state_command),
                );
            }

            HADMAP_ROUTE_MODE => {
                let mut hadmap_route_data_message = inputs
                    .remove(IN_HADMAP_ROUTE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let hadmap_route = hadmap_route_data_message
                    .data
                    .try_get::<AutowareAutoMsgsHadmapRoute>()?;
                local_planner_set_route(ptr, &hadmap_route);
            }

            VEHICLE_STATE_REPORT_MODE => {
                let mut vehicle_state_report_data_message = inputs
                    .remove(IN_VEHICLE_STATE_REPORT)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let vehicle_state_report = vehicle_state_report_data_message
                    .data
                    .try_get::<AutowareAutoMsgsVehicleStateReport>(
                )?;
                local_planner_set_state_report(ptr, &vehicle_state_report);
            }

            _ => {
                log::error!(
                    "Local Planner, unknown context mode: {}",
                    context.mode
                );
            }
        };

        Ok(results)
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
    Ok(Arc::new(LocalPlannerOperator) as Arc<dyn Operator>)
}
mod ffi;

use autoware_auto::msgs::ffi::{
    AutowareAutoMsgsHadmapRoute, AutowareAutoMsgsVehicleKinematicState,
    AutowareAutoMsgsVehicleStateReport,
};
use ffi::ffi::{
    get_state_cmd, get_trajectory, init_local_planner, set_kinematic_state, set_route,
    set_state_report, NativeConfig, Vehicle,
};
use ffi::NativeNodeInstance;
use std::{collections::HashMap, fmt::Debug, sync::Arc};
use zenoh_flow::{
    default_output_rule, export_operator, runtime::message::DataMessage,
    zenoh_flow_derive::ZFState, Configuration, Context, Data, DeadlineMiss, Node, NodeOutput,
    Operator, State, Token, ZFError, ZFResult,
};
use derive::{DefaultSendAndSync, zf_default_node};

static IN_VEHICLE_KINEMATIC_STATE: &str = "kinematic_state";
static IN_HADMAP_ROUTE: &str = "route";
static IN_VEHICLE_STATE_REPORT: &str = "state_report";
static OUT_TRAJECTORY: &str = "trajectory";
static OUT_VEHICLE_STATE_COMMAND: &str = "state_cmd";

const VEHICLE_KINEMATIC_STATE_MODE: usize = 1;
const HADMAP_ROUTE_MODE: usize = 2;
const VEHICLE_STATE_REPORT_MODE: usize = 3;


#[zf_default_node(init_fn="init_local_planner")]
#[derive(Debug, ZFState, DefaultSendAndSync)]
pub struct CustomNode;

impl Default for NativeConfig {
    fn default() -> Self {
        NativeConfig {
            enable_object_collision_estimator: false,
            heading_weight: 0.1,
            goal_distance_thresh: 3.0,
            stop_velocity_thresh: 2.0,
            subroute_goal_offset_lane2parking: 7.5,
            subroute_goal_offset_parking2lane: 7.5,
            vehicle: Vehicle {
                cg_to_front_m: 1.228,
                cg_to_rear_m: 1.5618,
                front_corner_stiffness: 0.1,
                rear_corner_stiffness: 0.1,
                mass_kg: 1500.0,
                yaw_inertia_kgm2: 12.0,
                width_m: 2.0,
                front_overhang_m: 1.05,
                rear_overhang_m: 0.92,
            },
        }
    }
}

fn get_config(configuration: &Option<Configuration>) -> NativeConfig {
    match configuration {
        Some(config) => {
            let enable_object_collision_estimator =
                match config["enable_object_collision_estimator"].as_bool() {
                    Some(v) => v,
                    None => NativeConfig::default().enable_object_collision_estimator,
                };
            let heading_weight = match config["heading_weight"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().heading_weight,
            };
            let goal_distance_thresh = match config["goal_distance_thresh"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().goal_distance_thresh,
            };
            let stop_velocity_thresh = match config["stop_velocity_thresh"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().stop_velocity_thresh,
            };
            let subroute_goal_offset_lane2parking =
                match config["subroute_goal_offset_lane2parking"].as_f64() {
                    Some(v) => v,
                    None => NativeConfig::default().subroute_goal_offset_lane2parking,
                };
            let subroute_goal_offset_parking2lane =
                match config["subroute_goal_offset_parking2lane"].as_f64() {
                    Some(v) => v,
                    None => NativeConfig::default().subroute_goal_offset_parking2lane,
                };
            let cg_to_front_m = match config["vehicle.cg_to_front_m"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vehicle.cg_to_front_m,
            };
            let cg_to_rear_m = match config["vehicle.cg_to_rear_m"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vehicle.cg_to_rear_m,
            };
            let front_corner_stiffness = match config["vehicle.front_corner_stiffness"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vehicle.front_corner_stiffness,
            };
            let rear_corner_stiffness = match config["vehicle.rear_corner_stiffness"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vehicle.rear_corner_stiffness,
            };
            let mass_kg = match config["vehicle.mass_kg"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vehicle.mass_kg,
            };
            let yaw_inertia_kgm2 = match config["vehicle.yaw_inertia_kgm2"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vehicle.yaw_inertia_kgm2,
            };
            let width_m = match config["vehicle.width_m"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vehicle.width_m,
            };
            let front_overhang_m = match config["vehicle.front_overhang_m"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vehicle.front_overhang_m,
            };
            let rear_overhang_m = match config["vehicle.rear_overhang_m"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().vehicle.rear_overhang_m,
            };
            let vehicle = Vehicle {
                cg_to_front_m,
                cg_to_rear_m,
                front_corner_stiffness,
                rear_corner_stiffness,
                mass_kg,
                yaw_inertia_kgm2,
                width_m,
                front_overhang_m,
                rear_overhang_m,
            };
            NativeConfig {
                enable_object_collision_estimator,
                heading_weight,
                goal_distance_thresh,
                stop_velocity_thresh,
                subroute_goal_offset_lane2parking,
                subroute_goal_offset_parking2lane,
                vehicle,
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
        let ptr = &mut dyn_state.try_get::<NativeNodeInstance>()?.ptr;

        match context.mode {
            VEHICLE_KINEMATIC_STATE_MODE => {
                let mut data_msg = inputs
                    .remove(IN_VEHICLE_KINEMATIC_STATE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let msg = data_msg
                    .data
                    .try_get::<AutowareAutoMsgsVehicleKinematicState>()?;
                set_kinematic_state(ptr, &msg);

                let trajectory = get_trajectory(ptr);
                let state_command = get_state_cmd(ptr);
                results.insert(OUT_TRAJECTORY.into(), Data::from(trajectory));
                results.insert(OUT_VEHICLE_STATE_COMMAND.into(), Data::from(state_command));
            }

            HADMAP_ROUTE_MODE => {
                let mut data_msg = inputs
                    .remove(IN_HADMAP_ROUTE)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let msg = data_msg.data.try_get::<AutowareAutoMsgsHadmapRoute>()?;
                set_route(ptr, &msg);
            }

            VEHICLE_STATE_REPORT_MODE => {
                let mut data_msg = inputs
                    .remove(IN_VEHICLE_STATE_REPORT)
                    .ok_or_else(|| ZFError::InvalidData("No data".to_string()))?;
                let msg = data_msg
                    .data
                    .try_get::<AutowareAutoMsgsVehicleStateReport>()?;
                set_state_report(ptr, &msg);
            }

            _ => {
                log::error!("Local Planner, unknown context mode: {}", context.mode);
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
    Ok(Arc::new(CustomNode) as Arc<dyn Operator>)
}

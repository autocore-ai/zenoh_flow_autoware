mod ffi;

use async_trait::async_trait;
use autoware_auto::NativeNodeInstance;
use common::built_in_types::ZFUsize;
use derive::ZenohFlowNode;
use ffi::ffi::{
    init, CommandBound, CommandBounds, NativeConfig, OptimizationWeights, StateBound, StateBounds,
    Vehicle,
};
use std::{fmt::Debug, sync::Arc, time::Duration};
use zenoh_flow::{
    async_std::task::sleep, export_source, types::ZFResult, zenoh_flow_derive::ZFState,
    Configuration, Context, Data, Node, Source, State,
};

#[derive(ZenohFlowNode, Debug, ZFState)]
pub struct CustomNode;

impl Default for NativeConfig {
    fn default() -> Self {
        NativeConfig {
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
            optimization_weights: OptimizationWeights {
                steering: 1.0,
                throttle: 1.0,
                goal: 0.5,
            },
            state_bounds: StateBounds {
                lower: StateBound {
                    x_m: -300.0,
                    y_m: -300.0,
                    velocity_mps: -3.0,
                    heading_rad: -6.2832,
                    steering_rad: -0.53,
                },
                upper: StateBound {
                    x_m: 300.0,
                    y_m: 300.0,
                    velocity_mps: 3.0,
                    heading_rad: 6.2832,
                    steering_rad: 0.53,
                },
            },
            command_bounds: CommandBounds {
                lower: CommandBound {
                    steering_rate_rps: -5.0,
                    throttle_mps2: -5.0,
                },
                upper: CommandBound {
                    steering_rate_rps: 5.0,
                    throttle_mps2: 5.0,
                },
            },
        }
    }
}

fn get_config(configuration: &Option<Configuration>) -> NativeConfig {
    match configuration {
        Some(config) => {
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
            let steering = match config["optimization_weights.steering"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().optimization_weights.steering,
            };
            let throttle = match config["optimization_weights.throttle"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().optimization_weights.throttle,
            };
            let goal = match config["optimization_weights.goal"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().optimization_weights.goal,
            };
            let optimization_weights = OptimizationWeights {
                steering,
                throttle,
                goal,
            };
            let x_m = match config["state_bounds.lower.x_m"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().state_bounds.lower.x_m,
            };
            let y_m = match config["state_bounds.lower.x_m"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().state_bounds.lower.y_m,
            };
            let velocity_mps = match config["state_bounds.lower.velocity_mps"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().state_bounds.lower.velocity_mps,
            };
            let heading_rad = match config["state_bounds.lower.heading_rad"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().state_bounds.lower.heading_rad,
            };
            let steering_rad = match config["state_bounds.lower.steering_rad"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().state_bounds.lower.steering_rad,
            };
            let lower = StateBound {
                x_m,
                y_m,
                velocity_mps,
                heading_rad,
                steering_rad,
            };
            let x_m = match config["state_bounds.upper.x_m"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().state_bounds.upper.x_m,
            };
            let y_m = match config["state_bounds.upper.x_m"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().state_bounds.upper.y_m,
            };
            let velocity_mps = match config["state_bounds.upper.velocity_mps"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().state_bounds.upper.velocity_mps,
            };
            let heading_rad = match config["state_bounds.upper.heading_rad"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().state_bounds.upper.heading_rad,
            };
            let steering_rad = match config["state_bounds.upper.steering_rad"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().state_bounds.upper.steering_rad,
            };
            let upper = StateBound {
                x_m,
                y_m,
                velocity_mps,
                heading_rad,
                steering_rad,
            };
            let state_bounds = StateBounds { lower, upper };
            let steering_rate_rps = match config["command_bounds.lower.steering_rate_rps"].as_f64()
            {
                Some(v) => v,
                None => {
                    NativeConfig::default()
                        .command_bounds
                        .lower
                        .steering_rate_rps
                }
            };
            let throttle_mps2 = match config["command_bounds.lower.throttle_mps2"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().command_bounds.lower.throttle_mps2,
            };
            let lower = CommandBound {
                steering_rate_rps,
                throttle_mps2,
            };
            let steering_rate_rps = match config["command_bounds.upper.steering_rate_rps"].as_f64()
            {
                Some(v) => v,
                None => {
                    NativeConfig::default()
                        .command_bounds
                        .upper
                        .steering_rate_rps
                }
            };
            let throttle_mps2 = match config["command_bounds.upper.throttle_mps2"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().command_bounds.upper.throttle_mps2,
            };
            let upper = CommandBound {
                steering_rate_rps,
                throttle_mps2,
            };
            let command_bounds = CommandBounds { lower, upper };
            NativeConfig {
                vehicle,
                optimization_weights,
                state_bounds,
                command_bounds,
            }
        }
        None => NativeConfig::default(),
    }
}

#[async_trait]
impl Source for CustomNode {
    async fn run(&self, _context: &mut Context, _dyn_state: &mut State) -> ZFResult<Data> {
        sleep(Duration::from_secs(1)).await;
        Ok(Data::from::<ZFUsize>(ZFUsize(1)))
    }
}

export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    Ok(Arc::new(CustomNode) as Arc<dyn Source>)
}

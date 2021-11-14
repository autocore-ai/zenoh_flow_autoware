mod ffi;

use async_trait::async_trait;
use autoware_auto::NativeNodeInstance;
use common::built_in_types::ZFUsize;
use derive::ZenohFlowNode;
use ffi::ffi::{init, GaussianSmoother, LanePlanner, NativeConfig, Vehicle};
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
            heading_weight: 0.1,
            lane_planner: LanePlanner {
                trajectory_resolution: 0.5,
            },
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
            gaussian_smoother: GaussianSmoother {
                standard_deviation: 5.0,
                kernel_size: 3,
            },
        }
    }
}

fn get_config(configuration: &Option<Configuration>) -> NativeConfig {
    match configuration {
        Some(config) => {
            let heading_weight = match config["heading_weight"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().heading_weight,
            };
            let trajectory_resolution = match config["lane_planner.trajectory_resolution"].as_f64()
            {
                Some(v) => v,
                None => NativeConfig::default().lane_planner.trajectory_resolution,
            };
            let lane_planner = LanePlanner {
                trajectory_resolution,
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
            let standard_deviation = match config["gaussian_smoother.standard_deviation"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().gaussian_smoother.standard_deviation,
            };
            let kernel_size = match config["gaussian_smoother.kernel_size"].as_i64() {
                Some(v) => v,
                None => NativeConfig::default().gaussian_smoother.kernel_size,
            };
            let gaussian_smoother = GaussianSmoother {
                standard_deviation,
                kernel_size,
            };
            NativeConfig {
                heading_weight,
                lane_planner,
                vehicle,
                gaussian_smoother,
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

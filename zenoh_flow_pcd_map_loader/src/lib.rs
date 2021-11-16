// Copyright 2021 The AutoCore.AI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

mod ffi;

use async_trait::async_trait;
use ffi::NativeNodeInstance;
use common::built_in_types::ZFString;
use ffi::ffi::{init_pcd_map_loader, MapConfig, NativeConfig, Vector3};
use std::{fmt::Debug, sync::Arc, time::Duration};
use zenoh_flow::{
    async_std::task::sleep, export_source, types::ZFResult, zenoh_flow_derive::ZFState,
    Configuration, Context, Data, Node, Source, State,
};

#[derive(Debug, ZFState)]
pub struct CustomNode;

unsafe impl Send for CustomNode {}
unsafe impl Sync for CustomNode {}

impl Node for CustomNode {
    fn initialize(&self, cfg: &Option<Configuration>) -> ZFResult<State> {
        Ok(State::from(NativeNodeInstance {
            ptr: init_pcd_map_loader(&get_config(cfg)),
        }))
    }
    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}


impl Default for NativeConfig {
    fn default() -> Self {
        NativeConfig {
            map_pcd_file: String::from(
                "/opt/AutowareAuto/share/autoware_demos/data/autonomoustuff_parking_lot_lgsvl.pcd",
            ),
            map_yaml_file: String::from(
                "/opt/AutowareAuto/share/autoware_demos/data/autonomoustuff_parking_lot_lgsvl.yaml",
            ),
            map_frame: String::from("map"),
            map_config: MapConfig {
                capacity: 1000000,
                min_point: Vector3 {
                    x: -1000.0,
                    y: -1000.0,
                    z: -3.0,
                },
                max_point: Vector3 {
                    x: 1000.0,
                    y: 1000.0,
                    z: 3.0,
                },
                voxel_size: Vector3 {
                    x: 3.5,
                    y: 3.5,
                    z: 3.5,
                },
            },
            viz_map: true,
        }
    }
}

fn get_config(configuration: &Option<Configuration>) -> NativeConfig {
    match configuration {
        Some(config) => {
            let map_pcd_file = match config["map_pcd_file"].as_str() {
                Some(v) => String::from(v),
                None => NativeConfig::default().map_pcd_file,
            };
            let map_yaml_file = match config["map_yaml_file"].as_str() {
                Some(v) => String::from(v),
                None => NativeConfig::default().map_yaml_file,
            };
            let map_frame = match config["map_frame"].as_str() {
                Some(v) => String::from(v),
                None => NativeConfig::default().map_frame,
            };
            let capacity = match config["map_config.capacity"].as_i64() {
                Some(v) => v,
                None => NativeConfig::default().map_config.capacity,
            };
            let x = match config["map_config.min_point.x"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().map_config.min_point.x,
            };
            let y = match config["map_config.min_point.y"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().map_config.min_point.y,
            };
            let z = match config["map_config.min_point.z"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().map_config.min_point.z,
            };
            let min_point = Vector3 { x, y, z };
            let x = match config["map_config.max_point.x"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().map_config.max_point.x,
            };
            let y = match config["map_config.max_point.y"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().map_config.max_point.y,
            };
            let z = match config["map_config.max_point.z"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().map_config.max_point.z,
            };
            let max_point = Vector3 { x, y, z };
            let x = match config["map_config.voxel_size.x"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().map_config.voxel_size.x,
            };
            let y = match config["map_config.voxel_size.y"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().map_config.voxel_size.y,
            };
            let z = match config["map_config.voxel_size.z"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().map_config.voxel_size.z,
            };
            let voxel_size = Vector3 { x, y, z };
            let map_config = MapConfig {
                capacity,
                min_point,
                max_point,
                voxel_size,
            };
            let viz_map = match config["viz_map"].as_bool() {
                Some(v) => v,
                None => NativeConfig::default().viz_map,
            };
            NativeConfig {
                map_pcd_file,
                map_yaml_file,
                map_frame,
                map_config,
                viz_map,
            }
        }
        None => NativeConfig::default(),
    }
}

#[async_trait]
impl Source for CustomNode {
    async fn run(&self, _context: &mut Context, _dyn_state: &mut State) -> ZFResult<Data> {
        sleep(Duration::from_secs(1)).await;
        Ok(Data::from::<ZFString>(ZFString(String::from("running"))))
    }
}

export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    Ok(Arc::new(CustomNode) as Arc<dyn Source>)
}

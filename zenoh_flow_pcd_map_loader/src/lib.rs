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

use async_trait::async_trait;
use cxx::UniquePtr;
use ffi::autoware_auto::ffi::{pcd_map_loader_init, CfgPcdMapLoader, PcdMapLoader};
use ffi::common_type::ZFString;
use std::time::Duration;
use std::{fmt::Debug, sync::Arc};
use zenoh_flow::{
    async_std::task::sleep, export_source, types::ZFResult, zenoh_flow_derive::ZFState,
    Configuration, Data, Node, Source, State, ZFError,
};

#[derive(Debug, ZFState)]
pub struct PcdMapLoaderSource;
unsafe impl Send for PcdMapLoaderSource {}
unsafe impl Sync for PcdMapLoaderSource {}
#[derive(Debug, ZFState)]
pub struct Instance {
    pub ptr: UniquePtr<PcdMapLoader>,
}
impl Node for PcdMapLoaderSource {
    fn initialize(&self, cfg: &Option<Configuration>) -> ZFResult<State> {
        let mut config = CfgPcdMapLoader::default();

        match cfg {
            Some(value) => {
                if let (
                    Some(map_pcd_file),
                    Some(map_yaml_file),
                    Some(map_frame),
                    Some(capacity),
                    Some(min_point_x),
                    Some(min_point_y),
                    Some(min_point_z),
                    Some(max_point_x),
                    Some(max_point_y),
                    Some(max_point_z),
                    Some(voxel_size_x),
                    Some(voxel_size_y),
                    Some(voxel_size_z),
                    Some(viz_map),
                ) = (
                    value.get("map_pcd_file"),
                    value.get("map_yaml_file"),
                    value.get("map_frame"),
                    value.get("map_config.capacity"),
                    value.get("map_config.min_point.x"),
                    value.get("map_config.min_point.y"),
                    value.get("map_config.min_point.z"),
                    value.get("map_config.max_point.x"),
                    value.get("map_config.max_point.y"),
                    value.get("map_config.max_point.z"),
                    value.get("map_config.voxel_size.x"),
                    value.get("map_config.voxel_size.y"),
                    value.get("map_config.voxel_size.z"),
                    value.get("viz_map"),
                ) {
                    config.map_pcd_file = serde_json::from_value(map_pcd_file.clone()).unwrap();
                    config.map_yaml_file = serde_json::from_value(map_yaml_file.clone()).unwrap();
                    config.map_frame = serde_json::from_value(map_frame.clone()).unwrap();
                    config.map_config.capacity = capacity.as_i64().unwrap();
                    config.map_config.min_point.x = min_point_x.as_f64().unwrap();
                    config.map_config.min_point.y = min_point_y.as_f64().unwrap();
                    config.map_config.min_point.z = min_point_z.as_f64().unwrap();
                    config.map_config.max_point.x = max_point_x.as_f64().unwrap();
                    config.map_config.max_point.y = max_point_y.as_f64().unwrap();
                    config.map_config.max_point.z = max_point_z.as_f64().unwrap();
                    config.map_config.voxel_size.x = voxel_size_x.as_f64().unwrap();
                    config.map_config.voxel_size.y = voxel_size_y.as_f64().unwrap();
                    config.map_config.voxel_size.z = voxel_size_z.as_f64().unwrap();
                    config.viz_map = viz_map.as_bool().unwrap();
                    log::info!("{:?}", config);
                } else {
                    log::warn!("Missing configuration!");
                    todo!("Use default config");
                    return Err(ZFError::MissingConfiguration);
                };
            }
            None => return Err(ZFError::MissingConfiguration),
        };
        let ptr = pcd_map_loader_init(&config);
        Ok(State::from(Instance { ptr }))
    }
    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}

#[async_trait]
impl Source for PcdMapLoaderSource {
    async fn run(
        &self,
        _context: &mut zenoh_flow::Context,
        _dyn_state: &mut State,
    ) -> ZFResult<Data> {
        sleep(Duration::from_secs(1)).await;
        Ok(Data::from::<ZFString>(ZFString(String::from("running"))))
    }
}

export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    env_logger::init();
    Ok(Arc::new(PcdMapLoaderSource) as Arc<dyn Source>)
}

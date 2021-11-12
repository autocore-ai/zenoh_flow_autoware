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
use ffi::autoware_auto::ffi::{osm_map_loader_init, CfgOsmMapLoader, OsmMapLoader};
use ffi::common_type::ZFString;
use std::time::Duration;
use std::{fmt::Debug, sync::Arc};
use zenoh_flow::{
    async_std::task::sleep, export_source, types::ZFResult, zenoh_flow_derive::ZFState,
    Configuration, Data, Node, Source, State, ZFError,
};

#[derive(Debug, ZFState)]
pub struct OsmMapLoaderSource;
unsafe impl Send for OsmMapLoaderSource {}
unsafe impl Sync for OsmMapLoaderSource {}
#[derive(Debug, ZFState)]
pub struct Instance {
    pub ptr: UniquePtr<OsmMapLoader>,
}
impl Node for OsmMapLoaderSource {
    fn initialize(&self, cfg: &Option<Configuration>) -> ZFResult<State> {
        let mut config = CfgOsmMapLoader::default();

        match cfg {
            Some(value) => {
                if let (
                    Some(map_osm_file),
                    Some(origin_offset_lat),
                    Some(origin_offset_lon),
                    Some(latitude),
                    Some(longitude),
                    Some(elevation),
                ) = (
                    value.get("map_osm_file"),
                    value.get("origin_offset_lat"),
                    value.get("origin_offset_lon"),
                    value.get("latitude"),
                    value.get("longitude"),
                    value.get("elevation"),
                ) {
                    config.map_osm_file = serde_json::from_value(map_osm_file.clone()).unwrap();
                    config.origin_offset_lat = origin_offset_lat.as_f64().unwrap();
                    config.origin_offset_lon = origin_offset_lon.as_f64().unwrap();
                    config.latitude = latitude.as_f64().unwrap();
                    config.longitude = longitude.as_f64().unwrap();
                    config.elevation = elevation.as_f64().unwrap();
                    log::info!("{:?}", config);
                } else {
                    log::warn!("Missing configuration!");
                    todo!("Use default config");
                    return Err(ZFError::MissingConfiguration);
                };
            }
            None => return Err(ZFError::MissingConfiguration),
        };
        let ptr = osm_map_loader_init(&config);
        Ok(State::from(Instance { ptr }))
    }
    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}

#[async_trait]
impl Source for OsmMapLoaderSource {
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
    Ok(Arc::new(OsmMapLoaderSource) as Arc<dyn Source>)
}

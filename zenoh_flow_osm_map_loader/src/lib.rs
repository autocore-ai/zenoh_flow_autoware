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
use autoware_auto::NativeNodeInstance;
use common::built_in_types::ZFString;
use derive::ZenohFlowNode;
use ffi::ffi::{init, NativeConfig};
use std::{fmt::Debug, sync::Arc, time::Duration};
use zenoh_flow::{
    async_std::task::sleep, export_source, types::ZFResult, zenoh_flow_derive::ZFState,
    Configuration, Context, Data, Node, Source, State,
};

#[derive(ZenohFlowNode, Debug, ZFState)]
pub struct CustomNode;

impl Default for NativeConfig {
    fn default() -> Self {
        println!("OK0");
        let ret = NativeConfig {
            map_osm_file: String::from(
                "/opt/AutowareAuto/share/autoware_demos/data/autonomoustuff_parking_lot.osm",
            ),
            origin_offset_lat: 0.1,
            origin_offset_lon: 0.1,
            latitude: 37.380811523812845,
            longitude: -121.90840595108715,
            elevation: 16.0,
        };
        println!("OK1");
        return ret;
    }
}

fn get_config(configuration: &Option<Configuration>) -> NativeConfig {
    println!("OKOKOK");
    let ret = match configuration {
        Some(config) => {
            
            println!("OKKKKKKK0");
            let map_osm_file = match config["map_osm_file"].as_str() {
                Some(v) => String::from(v),
                None => NativeConfig::default().map_osm_file,
            };
            let origin_offset_lat = match config["origin_offset_lat"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().origin_offset_lat,
            };
            let origin_offset_lon = match config["origin_offset_lon"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().origin_offset_lon,
            };
            let latitude = match config["latitude"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().latitude,
            };
            let longitude = match config["longitude"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().longitude,
            };
            let elevation = match config["elevation"].as_f64() {
                Some(v) => v,
                None => NativeConfig::default().elevation,
            };
            
            println!("OKKKKKKK1");
            NativeConfig {
                map_osm_file,
                origin_offset_lat,
                origin_offset_lon,
                latitude,
                longitude,
                elevation,
            }
        }
        None => NativeConfig::default(),
    };
    println!("ENDDDDDD");
    return ret;
}

#[async_trait]
impl Source for CustomNode {
    async fn run(&self, _context: &mut Context, _dyn_state: &mut State) -> ZFResult<Data> {
        println!("XXXXXXXXXXXZZZZZZZZZZZ");
        sleep(Duration::from_secs(1)).await;
        Ok(Data::from::<ZFString>(ZFString(String::from("running"))))
    }
}

export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    Ok(Arc::new(CustomNode) as Arc<dyn Source>)
}

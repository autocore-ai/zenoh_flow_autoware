mod ffi;
use async_trait::async_trait;
use ffi::ffi::{get_goal_pose, init_goal_pose, is_new_goal_pose, NativeConfig};
use ffi::NativeNodeInstance;
use std::{sync::Arc, time::Duration};
use zenoh_flow::{
    async_std::task::sleep, export_source, zenoh_flow_derive::ZFState, Configuration, Context,
    Data, Node, Source, State, ZFError, ZFResult,
};
use derive::{DefaultSendAndSync, zf_default_node};

#[zf_default_node(init_fn="init_goal_pose")]
#[derive(Debug, ZFState, DefaultSendAndSync)]
pub struct CustomNode;

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

#[async_trait]
impl Source for CustomNode {
    async fn run(&self, _context: &mut Context, dyn_state: &mut State) -> ZFResult<Data> {
        log::info!("zenoh flow goal pose running ...");
        let node = &mut dyn_state.try_get::<NativeNodeInstance>()?.ptr;
        let flag = true;
        while flag {
            if is_new_goal_pose(node) {
                return Ok(Data::from(get_goal_pose(node)));
            }
            sleep(Duration::from_secs(1)).await;
        }
        Err(ZFError::GenericError)
    }
}

export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    Ok(Arc::new(CustomNode) as Arc<dyn Source>)
}

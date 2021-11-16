mod ffi;
use async_trait::async_trait;
use ffi::NativeNodeInstance;
use ffi::ffi::{get_init_pose, init_init_pose, is_new, NativeConfig};
use std::{sync::Arc, time::Duration};
use zenoh_flow::{
    async_std::task::sleep, export_source, zenoh_flow_derive::ZFState, Configuration, Context,
    Data, Node, Source, State, ZFError, ZFResult,
};

#[derive(Debug, ZFState)]
pub struct CustomNode;

unsafe impl Send for CustomNode {}
unsafe impl Sync for CustomNode {}

impl Node for CustomNode {
    fn initialize(&self, cfg: &Option<Configuration>) -> ZFResult<State> {
        Ok(State::from(NativeNodeInstance {
            ptr: init_init_pose(&get_config(cfg)),
        }))
    }
    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}

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
        log::info!("zenoh flow init pose running ...");
        let node = &mut dyn_state.try_get::<NativeNodeInstance>()?.ptr;
        let flag = true;
        while flag {
            log::info!("Waiting init pose message ...");
            if is_new(node) {
                return Ok(Data::from(get_init_pose(node)));
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

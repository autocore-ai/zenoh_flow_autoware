use async_trait::async_trait;
use cxx::UniquePtr;
use ffi::autoware_auto::ffi::{get_init_pose, init_init_pose, is_new_init_pose, InitPose};
use std::sync::Arc;
use zenoh_flow::{
    export_source, zenoh_flow_derive::ZFState, Configuration, Context, Data, Node, Source, State,
    ZFError, ZFResult,
};

#[derive(Debug, ZFState)]
pub struct InitPoseSource;
unsafe impl Send for InitPoseSource {}
unsafe impl Sync for InitPoseSource {}

#[derive(Debug, ZFState)]
pub struct Instance {
    pub ptr: UniquePtr<InitPose>,
}

impl Node for InitPoseSource {
    fn initialize(&self, _configuration: &Option<Configuration>) -> ZFResult<State> {
        let ptr = init_init_pose();
        Ok(State::from(Instance { ptr }))
    }

    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}

#[async_trait]
impl Source for InitPoseSource {
    async fn run(&self, _context: &mut Context, dyn_state: &mut State) -> ZFResult<Data> {
        log::info!("zenoh flow init pose running ...");
        let wrapper = dyn_state.try_get::<Instance>()?;
        let node = &mut wrapper.ptr;

        let flag = true;
        while flag {
            log::info!("Waiting init pose message ...");
            if is_new_init_pose(node) {
                let init_pose = get_init_pose(node);
                log::info!("Receiver init pose: {:?}", init_pose);
                return Ok(Data::from(init_pose));
            }
            zenoh_flow::async_std::task::sleep(std::time::Duration::from_secs(1)).await;
        }
        Err(ZFError::GenericError)
    }
}

export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    env_logger::init();
    Ok(Arc::new(InitPoseSource) as Arc<dyn Source>)
}

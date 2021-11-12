use async_trait::async_trait;
use cxx::UniquePtr;
use ffi::autoware_auto::ffi::{
    goal_pose_get_goal_pose, goal_pose_init, goal_pose_is_new, GoalPose,
};
use std::sync::Arc;
use zenoh_flow::{
    export_source, zenoh_flow_derive::ZFState, Configuration, Context, Data, Node, Source, State,
    ZFError, ZFResult,
};

#[derive(Debug, ZFState)]
pub struct GoalPoseSource;
unsafe impl Send for GoalPoseSource {}
unsafe impl Sync for GoalPoseSource {}

#[derive(Debug, ZFState)]
pub struct Instance {
    pub ptr: UniquePtr<GoalPose>,
}

impl Node for GoalPoseSource {
    fn initialize(&self, _configuration: &Option<Configuration>) -> ZFResult<State> {
        let ptr = goal_pose_init();
        Ok(State::from(Instance { ptr }))
    }

    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}

#[async_trait]
impl Source for GoalPoseSource {
    async fn run(&self, _context: &mut Context, dyn_state: &mut State) -> ZFResult<Data> {
        log::info!("zenoh flow goal pose running ...");
        let wrapper = dyn_state.try_get::<Instance>()?;
        let node = &mut wrapper.ptr;

        let flag = true;
        while flag {
            log::info!("Waiting goal pose message ...");
            if goal_pose_is_new(node) {
                let goal_pose = goal_pose_get_goal_pose(node);
                log::info!("Receiver goal pose: {:?}", goal_pose);
                return Ok(Data::from(goal_pose));
            }
            zenoh_flow::async_std::task::sleep(std::time::Duration::from_secs(1)).await;
        }
        Err(ZFError::GenericError)
    }
}

export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    env_logger::init();
    Ok(Arc::new(GoalPoseSource) as Arc<dyn Source>)
}

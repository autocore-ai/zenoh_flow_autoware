
use async_trait::async_trait;
use std::sync::atomic::{AtomicUsize, Ordering};
use ffi::common_type::{ZFUsize};
use zenoh_flow::async_std::sync::Arc;
use zenoh_flow::{
    types::ZFResult, zenoh_flow_derive::ZFState, Configuration, Data, Node, Source, State, ZFError
};

static COUNTER: AtomicUsize = AtomicUsize::new(0);

#[derive(ZFState, Debug)]
struct HZState(pub u64);

#[derive(Debug, ZFState)]
struct TickSource;

#[async_trait]
impl Source for TickSource {
    async fn run(
        &self,
        _context: &mut zenoh_flow::Context,
        dyn_state: &mut State,
    ) -> ZFResult<Data> {
        let state = dyn_state.try_get::<HZState>()?;
        let sleep_time = 1000 / state.0 as u64;
        let d = ZFUsize(COUNTER.fetch_add(1, Ordering::AcqRel));
        zenoh_flow::async_std::task::sleep(std::time::Duration::from_millis(sleep_time)).await;
        Ok(Data::from::<ZFUsize>(d))
    }
}

impl Node for TickSource {
    fn initialize(&self, configuration: &Option<Configuration>) -> ZFResult<State> {
        if let Some(conf) = configuration {
            let hz = conf["hz"].as_u64().unwrap();
            Ok(State::from(HZState(hz)))
        } else {
            Err(ZFError::MissingConfiguration)
        }
    }

    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}

zenoh_flow::export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    env_logger::init();
    Ok(Arc::new(TickSource) as Arc<dyn Source>)
}
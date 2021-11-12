use async_trait::async_trait;
use zenoh_flow::async_std::sync::Arc;
use zenoh_flow::runtime::message::DataMessage;
use zenoh_flow::{Configuration,types::ZFResult, zf_empty_state, Node, Sink, State};

struct SystemMonitor;

#[async_trait]
impl Sink for SystemMonitor {
    async fn run(
        &self,
        _context: &mut zenoh_flow::Context,
        _dyn_state: &mut State,
        input: DataMessage,
    ) -> ZFResult<()> {
        log::info!("System Monitor received message: {:?}", input);
        Ok(())
    }
}

impl Node for SystemMonitor {
    fn initialize(&self, _configuration: &Option<Configuration>) -> ZFResult<State> {
        zf_empty_state!()
    }

    fn finalize(&self, _state: &mut State) -> ZFResult<()> {
        Ok(())
    }
}

zenoh_flow::export_sink!(register);

fn register() -> ZFResult<Arc<dyn Sink>> {
    env_logger::init();
    Ok(Arc::new(SystemMonitor) as Arc<dyn Sink>)
}
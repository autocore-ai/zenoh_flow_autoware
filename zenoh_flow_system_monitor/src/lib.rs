// Copyright 2021 The AutoCore.AI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use async_trait::async_trait;
use zenoh_flow::async_std::sync::Arc;
use zenoh_flow::runtime::message::DataMessage;
use zenoh_flow::{types::ZFResult, zf_empty_state, Configuration, Node, Sink, State};

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
    Ok(Arc::new(SystemMonitor) as Arc<dyn Sink>)
}

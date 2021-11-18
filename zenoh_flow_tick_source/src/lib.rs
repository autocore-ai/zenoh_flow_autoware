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
use common::built_in_types::ZFUsize;
use std::sync::atomic::{AtomicUsize, Ordering};
use zenoh_flow::async_std::sync::Arc;
use zenoh_flow::{
    types::ZFResult, zenoh_flow_derive::ZFState, Configuration, Data, Node, Source, State, ZFError,
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
        log::debug!("tick source output: {:?}", d);
        Ok(Data::from::<ZFUsize>(d))
    }
}

impl Node for TickSource {
    fn initialize(&self, configuration: &Option<Configuration>) -> ZFResult<State> {
        if let Some(conf) = configuration {
            let hz = conf["hz"].as_u64().unwrap();
            log::debug!("HZ of Tick Source is {:?}", hz);
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
    Ok(Arc::new(TickSource) as Arc<dyn Source>)
}

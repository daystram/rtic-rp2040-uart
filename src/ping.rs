use alloc::vec::Vec;
use core::cell::RefCell;
use defmt::{info, trace, Format};
use rtic_monotonics::{rtic_time::monotonic::TimerQueueBasedInstant, Monotonic};
use serde::{Deserialize, Serialize};

use crate::{
    kb::Mono,
    transport::{FunctionId, RemoteInvoker},
};

pub struct Ping {
    counter: u32,
}

pub const FUNCTION_ID_PING: FunctionId = 0x01;

impl Ping {
    pub fn new() -> Self {
        Ping { counter: 0 }
    }

    pub async fn ping<I>(&mut self, client: &RefCell<I>)
    where
        I: RemoteInvoker,
    {
        self.counter = self.counter.wrapping_add(1);

        let req = PingRequest {
            counter: self.counter,
            tick: Mono::now().ticks(),
        };
        info!("ping(): request: [{:?}]", req);

        let start_time = Mono::now();
        let res = client
            .borrow_mut()
            .invoke::<PingRequest, PingResponse>(FUNCTION_ID_PING, req)
            .await;
        let end_time = Mono::now();

        info!(
            "ping(): response=[{:?}] (latency={}us)",
            res,
            (end_time - start_time).to_micros()
        );
    }

    pub fn ping_remote(req_buffer: &[u8]) -> Vec<u8> {
        info!("ping_remote()");
        let req: PingRequest = postcard::from_bytes(req_buffer).unwrap();
        let res = PingResponse {
            counter: req.counter,
            tick: Mono::now().ticks(),
        };
        trace!(
            "allocating: struct={}B res={}B",
            size_of::<PingResponse>(),
            size_of_val(&res)
        );
        postcard::to_allocvec(&res).unwrap()
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
struct PingRequest {
    counter: u32,
    tick: u64,
}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
pub struct PingResponse {
    pub counter: u32,
    pub tick: u64,
}

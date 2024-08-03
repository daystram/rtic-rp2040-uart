use alloc::{borrow::ToOwned, format, string::String, vec::Vec};
use core::cell::RefCell;
use defmt::{error, info, trace, write, Format};
use rtic_monotonics::{rtic_time::monotonic::TimerQueueBasedInstant, Monotonic};
use serde::{Deserialize, Serialize};

use crate::{
    kb::Mono,
    remote::{MethodId, RemoteInvoker, Service, ServiceId},
};

pub struct Ping {
    counter: u32,
}

pub const SERVICE_ID_PING: ServiceId = 0x01;
pub const METHOD_ID_PING_A: MethodId = 0x01;
pub const METHOD_ID_PING_B: MethodId = 0x02;

impl Ping {
    pub fn new() -> Self {
        Ping { counter: 0 }
    }

    pub async fn ping_a<I>(&mut self, client: &RefCell<I>)
    where
        I: RemoteInvoker,
    {
        self.counter = self.counter.wrapping_add(1);

        let req = PingARequest {
            counter: self.counter,
            tick: Mono::now().ticks(),
        };
        // info!("ping_a(): request: [{:?}]", req);

        let res = client
            .borrow_mut()
            .invoke::<PingARequest, PingAResponse>(SERVICE_ID_PING, METHOD_ID_PING_A, req)
            .await;
        // info!("ping_a(): response=[{:?}]", res,);
    }

    fn service_ping_a(&mut self, request: PingARequest) -> PingAResponse {
        // info!("service_ping_a()");
        PingAResponse {
            counter: request.counter + 10,
            tick: Mono::now().ticks(),
        }
    }

    pub async fn ping_b<I>(&mut self, client: &RefCell<I>)
    where
        I: RemoteInvoker,
    {
        self.counter = self.counter.wrapping_add(1);

        let req = PingBRequest {
            text: format!("hellooooooooooooooooo {}", self.counter),
            counter: self.counter,
            tick: Mono::now().ticks(),
        };
        // info!("ping_b(): request: [{:?}]", req);

        let res = client
            .borrow_mut()
            .invoke::<PingBRequest, PingBResponse>(SERVICE_ID_PING, METHOD_ID_PING_B, req)
            .await;
        // info!("ping_b(): response=[{:?}]", res,);
    }

    fn service_ping_b(&mut self, request: PingBRequest) -> PingBResponse {
        // info!("service_ping_b()");
        PingBResponse {
            text: format!("bye, {}", request.text),
            counter: request.counter + 100,
            tick: Mono::now().ticks(),
        }
    }
}

impl Service for Ping {
    fn dispatch(&mut self, method_id: MethodId, request_buffer: &[u8]) -> Vec<u8> {
        // info!("Ping::dispatch()");
        match method_id {
            METHOD_ID_PING_A => {
                let request: PingARequest = postcard::from_bytes(request_buffer).unwrap();
                let response = self.service_ping_a(request);
                postcard::to_allocvec(&response).unwrap()
            }
            METHOD_ID_PING_B => {
                let request: PingBRequest = postcard::from_bytes(request_buffer).unwrap();
                let response = self.service_ping_b(request);
                postcard::to_allocvec(&response).unwrap()
            }
            _ => {
                error!("unimplemented");
                Vec::new()
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
struct PingARequest {
    counter: u32,
    tick: u64,
}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
pub struct PingAResponse {
    counter: u32,
    tick: u64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct PingBRequest {
    text: String,
    counter: u32,
    tick: u64,
}

impl Format for PingBRequest {
    fn format(&self, fmt: defmt::Formatter) {
        write!(
            fmt,
            "PingBRequest {{ text: \"{}\", counter: {}, tick: {} }}",
            self.text.as_str(),
            self.counter,
            self.tick
        );
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct PingBResponse {
    text: String,
    counter: u32,
    tick: u64,
}

impl Format for PingBResponse {
    fn format(&self, fmt: defmt::Formatter) {
        write!(
            fmt,
            "PingBResponse {{ text: \"{}\", counter: {}, tick: {} }}",
            self.text.as_str(),
            self.counter,
            self.tick
        );
    }
}

pub mod transport;

use alloc::{boxed::Box, vec::Vec};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use rtic_monotonics::Monotonic;
use rtic_sync::channel::Receiver;
use serde::{Deserialize, Serialize};
use transport::{Sequence, TransportSender};

use crate::{kb::Mono, util};

pub const REQUEST_SEQUENCE_QUEUE_SIZE: usize = 1;

static SERVICE_REGISTRY: Mutex<RefCell<[Option<Box<dyn Service>>; ServiceId::MAX as usize + 1]>> =
    Mutex::new(RefCell::new([const { None }; ServiceId::MAX as usize + 1]));

pub type ServiceId = u8;
pub type MethodId = u8;

pub trait Service: Send + Sync {
    fn get_service_id(&self) -> ServiceId;
    fn dispatch(&mut self, method_id: MethodId, request: &[u8]) -> Vec<u8>;
}

pub struct Executor {
    seq_receiver: Receiver<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE>,
}

impl Executor {
    pub fn new(seq_receiver: Receiver<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE>) -> Self {
        Executor { seq_receiver }
    }

    pub fn register_service(&mut self, service: Box<dyn Service>) {
        let service_id = service.get_service_id();
        cortex_m::interrupt::free(|cs| {
            SERVICE_REGISTRY.borrow(cs).borrow_mut()[service_id as usize] = Some(service);
        });
    }

    pub async fn listen<S>(&mut self, sender: &RefCell<S>)
    where
        S: TransportSender,
    {
        while let Ok(sequence) = self.seq_receiver.recv().await {
            let start_time = Mono::now();
            let mut client = sender.borrow_mut();

            // retrieve request
            let (service_id, method_id, req, mut respond) = match client.get_payload(sequence) {
                Ok(r) => r,
                Err(err) => {
                    defmt::error!("failed to retrieve request payload: {}", err);
                    return;
                }
            };

            // execute function
            let res = match cortex_m::interrupt::free(|cs| {
                SERVICE_REGISTRY.borrow(cs).borrow_mut()[service_id as usize]
                    .as_mut()
                    .map(|service| service.dispatch(method_id, req))
            }) {
                Some(res) => res,
                None => {
                    defmt::error!("service not implemented: service_id={}", service_id);
                    return;
                }
            };

            // return response
            respond(&res);

            let end_time = Mono::now();
            util::log_duration(util::LogDurationTag::ServerLatency, start_time, end_time);
            util::log_heap();
        }
    }
}

pub trait RemoteInvoker {
    fn invoke<'b, Q, R>(
        &mut self,
        service_id: ServiceId,
        method_id: MethodId,
        request: Q,
    ) -> impl core::future::Future<Output = R>
    where
        Q: Serialize,
        R: Deserialize<'b>;
}

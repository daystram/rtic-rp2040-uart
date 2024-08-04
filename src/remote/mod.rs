pub mod transport;

use alloc::{boxed::Box, rc::Rc, vec::Vec};
use async_trait::async_trait;
use core::cell::{LazyCell, RefCell};
use defmt::Format;
use rtic_monotonics::Monotonic;
use rtic_sync::{arbiter::Arbiter, channel::Receiver};
use serde::{Deserialize, Serialize};
use transport::{Sequence, TransportSender};

use crate::{kb::Mono, util};

pub const REQUEST_SEQUENCE_QUEUE_SIZE: usize = 1;

static SERVICE_REGISTRY: Arbiter<
    LazyCell<Rc<RefCell<[Option<Box<dyn Service>>; ServiceId::MAX as usize + 1]>>>,
> = Arbiter::new(LazyCell::new(|| {
    Rc::new(RefCell::new([const { None }; ServiceId::MAX as usize + 1]))
}));

pub type ServiceId = u8;
pub type MethodId = u8;

#[async_trait]
pub trait Service: Send + Sync {
    fn get_service_id(&self) -> ServiceId;
    async fn dispatch(&mut self, method_id: MethodId, request: &[u8]) -> Result<Vec<u8>, Error>;
}

#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum Error {
    ResponseSerializationFailed,
    MethodUnimplemented,
}

pub struct Executor {
    seq_receiver: Receiver<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE>,
}

impl Executor {
    pub fn new(seq_receiver: Receiver<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE>) -> Self {
        Executor { seq_receiver }
    }

    pub async fn register_service(&mut self, service: Box<dyn Service>) {
        let service_id = service.get_service_id();
        SERVICE_REGISTRY.access().await.as_ref().borrow_mut()[service_id as usize] = Some(service);
    }

    pub async fn listen<S>(&mut self, sender: &Arbiter<Rc<RefCell<S>>>)
    where
        S: TransportSender,
    {
        while let Ok(sequence) = self.seq_receiver.recv().await {
            let start_time = Mono::now();
            let c = sender.access().await;
            let mut client = c.as_ref().borrow_mut();

            // retrieve request
            let (service_id, method_id, req, mut respond) = match client.get_payload(sequence) {
                Ok(r) => r,
                Err(err) => {
                    defmt::error!("failed to retrieve request payload: {}", err);
                    return;
                }
            };

            // execute
            let res = match match SERVICE_REGISTRY.access().await.as_ref().borrow_mut()
                [service_id as usize]
            {
                Some(ref mut service) => service.dispatch(method_id, req).await,
                None => {
                    defmt::error!("service not implemented: service_id={}", service_id);
                    return;
                }
            } {
                Ok(res) => res,
                Err(err) => {
                    defmt::error!("failed to execute method: {}", err);
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

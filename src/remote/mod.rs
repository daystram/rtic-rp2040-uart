pub mod transport;

use alloc::{boxed::Box, vec::Vec};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use defmt::error;
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
    fn dispatch(&mut self, method_id: MethodId, request: &[u8]) -> Vec<u8>;
}

pub struct Executor {
    seq_receiver: Receiver<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE>,
}

impl Executor {
    pub fn new(seq_receiver: Receiver<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE>) -> Self {
        Executor { seq_receiver }
    }

    pub fn register_service(&mut self, fun: MethodId, handler: Box<dyn Service>) {
        cortex_m::interrupt::free(|cs| {
            SERVICE_REGISTRY.borrow(cs).borrow_mut()[fun as usize] = Some(handler);
        });
    }

    pub async fn listen<S>(&mut self, client: &RefCell<S>)
    where
        S: TransportSender,
    {
        while let Ok(seq) = self.seq_receiver.recv().await {
            let start_time = Mono::now();

            // parse request payload
            let packet = match cortex_m::interrupt::free(|cs| {
                transport::PACKET_BUFFER.borrow(cs).borrow()[seq as usize]
            }) {
                Some(packet) => {
                    // TODO: dropping here allows the freeing of heap early, but this somehow break packet.payload
                    // warn!(
                    //     "dropping on listen: size={}B",
                    //     mem::size_of_val(packet.payload)
                    // );
                    // drop(unsafe { Box::from_raw(packet.payload as *const [u8] as *mut [u8]) });
                    packet
                }
                None => {
                    error!("packet not found in buffer: seq={}", seq);
                    return;
                }
            };

            // execute function
            let res = cortex_m::interrupt::free(|cs| {
                if let Some(ref mut service) =
                    SERVICE_REGISTRY.borrow(cs).borrow_mut()[packet.service_id as usize]
                {
                    service.dispatch(packet.method_id, packet.payload)
                } else {
                    error!("service not implemented: service_id={}", packet.service_id);
                    Vec::new()
                }
            });

            // return response
            client.borrow_mut().send_response(
                packet.sequence,
                packet.service_id,
                packet.method_id,
                res.as_slice(),
            );

            let end_time = Mono::now();
            util::log_duration(util::LogDurationTag::ServerLatency, start_time, end_time);

            util::log_heap();
        }
    }
}

pub trait RemoteInvoker {
    async fn invoke<'b, Q, R>(
        &mut self,
        service_id: ServiceId,
        method_id: MethodId,
        request: Q,
    ) -> R
    where
        Q: Serialize,
        R: Deserialize<'b>;
}

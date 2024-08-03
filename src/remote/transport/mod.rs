pub mod uart;

use core::{cell::RefCell, future::Future};

use alloc::vec::Vec;
use cortex_m::interrupt::Mutex;
use defmt::Format;
use rtic_sync::channel::Receiver;
use serde::{Deserialize, Serialize};

pub static PACKET_BUFFER: Mutex<RefCell<[Option<Packet>; Sequence::MAX as usize + 1]>> =
    Mutex::new(RefCell::new([None; Sequence::MAX as usize + 1]));

#[derive(Clone, Copy, Debug, Deserialize, Format, PartialEq, Serialize)]
#[repr(u8)]
enum Kind {
    Request,
    Response,
}

pub type Sequence = u8;

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
pub struct Packet<'a> {
    pub kind: Kind,
    pub sequence: Sequence,
    pub service_id: super::ServiceId,
    pub method_id: super::MethodId,
    pub payload: &'a [u8],
}

/*
    Packet
         0   1   2   3   4   5   6   7
       +---+---+---+---+---+---+---+---+
    0  |            length             |
       +---+---+---+---+---+---+---+---+
    8  |             kind              |
       +---+---+---+---+---+---+---+---+
   16  |           sequence            |
       +---+---+---+---+---+---+---+---+
   32  |   service_id  |   method_id   |
       +---+---+---+---+---+---+---+---+
   64  |                               |
    .  |                               |
    .  |           payload             |
    .  |                               |
    N  |                               |
       +---+---+---+---+---+---+---+---+
*/

pub trait TransportReceiver {
    fn initialize_seq_sender(
        &mut self,
    ) -> Receiver<'static, Sequence, { super::REQUEST_SEQUENCE_QUEUE_SIZE }>;
    fn read_into_buffer(&mut self);
}

pub trait TransportSender {
    fn send_request(
        &mut self,
        service_id: super::ServiceId,
        method_id: super::MethodId,
        payload: &[u8],
    ) -> Sequence;
    fn send_response(
        &mut self,
        sequence: Sequence,
        service_id: super::ServiceId,
        method_id: super::MethodId,
        payload: &[u8],
    );
    fn receive_response_poll(&mut self, sequence: Sequence)
        -> impl Future<Output = Vec<u8>> + Send;
}

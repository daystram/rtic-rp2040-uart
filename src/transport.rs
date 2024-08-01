use alloc::{borrow::ToOwned, boxed::Box, vec::Vec};
use core::{any::Any, cell::RefCell, error, future::Future, mem};
use cortex_m::interrupt::Mutex;
use defmt::{debug, error, trace, warn, Format};
use embedded_io::{Read, Write};
use hal::{
    gpio,
    uart::{Reader, Writer},
};
use rtic_monotonics::{rp2040::prelude::*, Monotonic};
use rtic_sync::channel::{Receiver, Sender};
use serde::{Deserialize, Serialize};

use crate::kb::{Mono, HEAP};

const UART_FRAME_BUFFER_SIZE_BYTES: usize = 256;
// const UART_FRAME_BUFFER_SIZE_BYTES: usize = 1 + (2 >> u8::BITS);

const REQUEST_SEQUENCE_QUEUE_SIZE: usize = 1;

static PACKET_BUFFER: Mutex<RefCell<[Option<Packet>; Sequence::MAX as usize + 1]>> =
    Mutex::new(RefCell::new([None; Sequence::MAX as usize + 1]));

static SERVICE_REGISTRY: Mutex<RefCell<[Option<Box<dyn Service>>; ServiceId::MAX as usize + 1]>> =
    Mutex::new(RefCell::new([const { None }; ServiceId::MAX as usize + 1]));

pub trait Service: Send + Sync {
    fn dispatch(&mut self, method_id: MethodId, request: &[u8]) -> Vec<u8>;
}

pub struct RemoteExecutor {
    seq_receiver: Receiver<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE>,
}

impl RemoteExecutor {
    pub fn new(seq_receiver: Receiver<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE>) -> Self {
        RemoteExecutor { seq_receiver }
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
            // parse request payload
            let packet = match cortex_m::interrupt::free(|cs| {
                PACKET_BUFFER.borrow(cs).borrow()[seq as usize]
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
            warn!("packet.payload: {}", packet.payload);

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

            // error!(
            //     "========= heap stat: free={}B used={}B",
            //     HEAP.free(),
            //     HEAP.used()
            // );
        }
    }
}

pub trait TransportReceiver {
    fn initialize_seq_sender(&mut self)
        -> Receiver<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE>;
    fn read_into_buffer(&mut self);
}

pub struct UartReceiver {
    uart_reader: Reader<
        hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >,
    seq_sender: Option<Sender<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE>>,
}

impl UartReceiver {
    pub fn new(
        mut uart_reader: Reader<
            hal::pac::UART0,
            (
                gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
                gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
            ),
        >,
    ) -> Self {
        uart_reader.enable_rx_interrupt();
        UartReceiver {
            uart_reader,
            seq_sender: None,
        }
    }
}

impl TransportReceiver for UartReceiver {
    fn initialize_seq_sender(
        &mut self,
    ) -> Receiver<'static, Sequence, REQUEST_SEQUENCE_QUEUE_SIZE> {
        let (seq_sender, seq_receiver) =
            rtic_sync::make_channel!(Sequence, REQUEST_SEQUENCE_QUEUE_SIZE);
        self.seq_sender = Some(seq_sender);
        seq_receiver
    }

    fn read_into_buffer(&mut self) {
        let mut buffer = [0u8; UART_FRAME_BUFFER_SIZE_BYTES];
        let mut offset = 0;

        match self.uart_reader.read_raw(&mut buffer[offset..]) {
            Ok(n) => {
                offset += n;
            }
            Err(_) => return, // fails on first read, drop
        }

        let expected_length = buffer[0] as usize;
        if expected_length == 0 {
            return;
        }
        if expected_length > buffer.len() {
            error!(
                "expected packet overflows buffer: expected_len={}",
                expected_length
            )
        }

        while offset < expected_length {
            match self.uart_reader.read_raw(&mut buffer[offset..]) {
                Ok(n) => {
                    offset += n;
                }
                Err(nb::Error::WouldBlock) => continue,
                Err(_) => return, // unhandled
            }
        }
        if offset == 0 {
            return;
        }
        let packet: Packet =
            postcard::from_bytes_cobs(&mut buffer[1..expected_length + 1]).unwrap();

        error!(
            "========= 1 heap stat: free={}B used={}B",
            HEAP.free(),
            HEAP.used()
        );
        cortex_m::interrupt::free(|cs| {
            if let Some(ref mut old_packet) =
                PACKET_BUFFER.borrow(cs).borrow_mut()[packet.sequence as usize]
            {
                warn!("dropping on wrap: size={}B", mem::size_of_val(old_packet));
                drop(unsafe { Box::from_raw(old_packet.payload as *const [u8] as *mut [u8]) });
            }
            PACKET_BUFFER.borrow(cs).borrow_mut()[packet.sequence as usize] = Some(Packet {
                kind: packet.kind,
                sequence: packet.sequence,
                service_id: packet.service_id,
                method_id: packet.method_id,
                payload: Box::leak(packet.payload.to_owned().into_boxed_slice()),
            });
        });
        error!(
            "========= 2 heap stat: free={}B used={}B",
            HEAP.free(),
            HEAP.used()
        );

        if packet.kind == Kind::Request {
            if let Some(ref mut seq_sender) = self.seq_sender {
                if seq_sender.try_send(packet.sequence).is_err() {
                    error!(
                        "request sequence queue is full, request dropped: seq={}",
                        packet.sequence
                    );
                };
            }
        }
    }
}

pub trait TransportSender {
    fn send_request(
        &mut self,
        service_id: ServiceId,
        method_id: MethodId,
        payload: &[u8],
    ) -> Sequence;
    fn send_response(
        &mut self,
        sequence: Sequence,
        service_id: ServiceId,
        method_id: MethodId,
        payload: &[u8],
    );
    fn receive_response_poll(&mut self, sequence: Sequence)
        -> impl Future<Output = Vec<u8>> + Send;
}

pub struct UartSender {
    uart_writer: Writer<
        hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >,
}

impl UartSender {
    pub fn new(
        mut uart_writer: Writer<
            hal::pac::UART0,
            (
                gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
                gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
            ),
        >,
    ) -> Self {
        uart_writer.disable_tx_interrupt();
        UartSender { uart_writer }
    }

    fn get_next_sequence() -> Sequence {
        static mut SEQUENCE: Sequence = 0;
        unsafe {
            SEQUENCE = SEQUENCE.wrapping_add(1);
            SEQUENCE
        }
    }
}

impl TransportSender for UartSender {
    fn send_request(
        &mut self,
        service_id: ServiceId,
        method_id: MethodId,
        payload: &[u8],
    ) -> Sequence {
        let sequence = Self::get_next_sequence();

        let packet = postcard::to_allocvec_cobs(&Packet {
            kind: Kind::Request,
            sequence,
            service_id,
            method_id,
            payload,
        })
        .unwrap();

        let mut buffer = Vec::with_capacity(packet.len() + 1);
        buffer.push(packet.len() as u8);
        buffer.extend_from_slice(&packet);

        self.uart_writer.write_all(&buffer).unwrap();
        sequence
    }

    fn send_response(
        &mut self,
        sequence: Sequence,
        service_id: ServiceId,
        method_id: MethodId,
        payload: &[u8],
    ) {
        let packet = postcard::to_allocvec_cobs(&Packet {
            kind: Kind::Response,
            sequence,
            service_id,
            method_id,
            payload,
        })
        .unwrap();

        let mut buffer = Vec::with_capacity(packet.len() + 1);
        buffer.push(packet.len() as u8);
        buffer.extend_from_slice(&packet);

        self.uart_writer.write_all(&buffer).unwrap();
    }

    async fn receive_response_poll(&mut self, seq: Sequence) -> Vec<u8> {
        let stored_payload = loop {
            if let Some(response) = cortex_m::interrupt::free(|cs| {
                PACKET_BUFFER.borrow(cs).borrow_mut()[seq as usize].take()
            }) {
                break response.payload;
            }
            Mono::delay(100.micros()).await;
        };

        let payload = stored_payload.to_owned();
        drop(unsafe { Box::from_raw(stored_payload as *const [u8] as *mut [u8]) });
        payload
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

impl RemoteInvoker for UartSender {
    async fn invoke<'b, Q, R>(
        &mut self,
        service_id: ServiceId,
        method_id: MethodId,
        request: Q,
    ) -> R
    where
        Q: Serialize,
        R: Deserialize<'b>,
    {
        let mut request_buffer = [0u8; UART_FRAME_BUFFER_SIZE_BYTES];
        let request_payload = postcard::to_slice(&request, &mut request_buffer).unwrap();

        let seq = self.send_request(service_id, method_id, request_payload);

        let response_buffer = self.receive_response_poll(seq).await.leak();
        let response_payload = postcard::from_bytes(response_buffer).unwrap();
        drop(unsafe { Box::from_raw(response_buffer as *const [u8] as *mut [u8]) });

        response_payload
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Format, PartialEq, Serialize)]
#[repr(u8)]
pub enum Kind {
    Request,
    Response,
}

pub type Sequence = u8;
pub type ServiceId = u8;
pub type MethodId = u8;

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
pub struct Packet<'a> {
    kind: Kind,
    sequence: Sequence,
    service_id: ServiceId,
    method_id: MethodId,
    payload: &'a [u8],
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

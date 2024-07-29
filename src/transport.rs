use alloc::{boxed::Box, vec::Vec};
use core::{cell::RefCell, future::Future};
use cortex_m::interrupt::Mutex;
use defmt::{error, trace, Format};
use embedded_io::Write;
use hal::{
    gpio,
    uart::{self, Reader, UartPeripheral, Writer},
};
use rtic_monotonics::{rp2040::prelude::*, Monotonic};
use rtic_sync::channel::{Receiver, Sender};
use serde::{Deserialize, Serialize};

use crate::{kb::Mono, ping::PingResponse};

const UART_BUFFER_SIZE_BYTES: usize = 256;

const PACKET_QUEUE_SIZE: usize = 1;

static PACKET_BUFFER: Mutex<RefCell<[Option<Packet>; Sequence::MAX as usize + 1]>> =
    Mutex::new(RefCell::new([None; Sequence::MAX as usize + 1]));
static FUNCTION_REGISTRY: Mutex<RefCell<[Option<Function>; FunctionId::MAX as usize + 1]>> =
    Mutex::new(RefCell::new([None; FunctionId::MAX as usize + 1]));

type Function = fn(&[u8]) -> Vec<u8>;

// pub trait Function {
//     fn handle<'a, Q, R>(&mut self, request: Q) -> R
//     where
//         Q: Deserialize<'a> + Format,
//         R: Serialize + Format;
// }

pub struct RemoteExecutor {
    seq_receiver: Receiver<'static, Sequence, PACKET_QUEUE_SIZE>,
}

impl RemoteExecutor {
    pub fn new(seq_receiver: Receiver<'static, Sequence, PACKET_QUEUE_SIZE>) -> Self {
        RemoteExecutor { seq_receiver }
    }

    // TODO: consider how to accept an impl method for mutable reference
    pub fn register_function(&mut self, fun: FunctionId, function: Function) {
        cortex_m::interrupt::free(|cs| {
            FUNCTION_REGISTRY.borrow(cs).borrow_mut()[fun as usize] = Some(function);
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
                Some(packet) => packet,
                None => {
                    error!("packet not found in buffer: seq={}", seq);
                    return;
                }
            };
            let function = match cortex_m::interrupt::free(|cs| {
                FUNCTION_REGISTRY.borrow(cs).borrow()[packet.fun as usize]
            }) {
                Some(function) => function,
                None => {
                    error!(
                        "function handler not registered: seq={} fun={}",
                        packet.seq, packet.fun
                    );
                    return;
                }
            };

            // execute function
            let res = function(packet.payload);

            // return response
            client
                .borrow_mut()
                .send_response(packet.seq, packet.fun, res.as_slice());
        }
    }
}

pub trait TransportReceiver {
    fn initialize_seq_sender(&mut self) -> Receiver<'static, Sequence, PACKET_QUEUE_SIZE>;
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
    seq_sender: Option<Sender<'static, Sequence, PACKET_QUEUE_SIZE>>,
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
    fn initialize_seq_sender(&mut self) -> Receiver<'static, Sequence, PACKET_QUEUE_SIZE> {
        let (seq_sender, seq_receiver) = rtic_sync::make_channel!(Sequence, PACKET_QUEUE_SIZE);
        self.seq_sender = Some(seq_sender);
        seq_receiver
    }

    fn read_into_buffer(&mut self) {
        let mut buffer = [0u8; 1024];
        if self.uart_reader.read_raw(&mut buffer).is_err() {
            return;
        }
        let packet: Packet = postcard::from_bytes(&buffer).unwrap();
        trace!("packet: [{}]", packet);

        let payload: PingResponse = postcard::from_bytes(packet.payload).unwrap();
        trace!("packet.payload: [{}]", payload);

        cortex_m::interrupt::free(|cs| {
            PACKET_BUFFER.borrow(cs).borrow_mut()[packet.seq as usize] = Some(Packet {
                typ: packet.typ,
                seq: packet.seq,
                fun: packet.fun,
                payload: Box::leak(packet.payload.to_vec().into_boxed_slice()),
            });
        });

        if packet.typ == Type::Request {
            if let Some(ref mut seq_sender) = self.seq_sender {
                if seq_sender.try_send(packet.seq).is_err() {
                    error!("request queue is full, request dropped: seq={}", packet.seq);
                };
            }
        }
    }
}

pub trait TransportSender {
    fn send_request(&mut self, fun: FunctionId, payload: &[u8]) -> Sequence;
    fn send_response(&mut self, seq: Sequence, fun: FunctionId, payload: &[u8]);
    fn receive_respone_poll<'a>(&mut self, seq: Sequence) -> impl Future<Output = &'a [u8]> + Send;
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

    fn get_seq() -> Sequence {
        static mut SEQ: Sequence = 0;
        unsafe {
            SEQ = SEQ.wrapping_add(1);
            SEQ
        }
    }
}

impl TransportSender for UartSender {
    fn send_request(&mut self, fun: FunctionId, payload: &[u8]) -> Sequence {
        let seq = Self::get_seq();
        let mut buffer = [0u8; UART_BUFFER_SIZE_BYTES];
        self.uart_writer
            .write(
                postcard::to_slice(
                    &Packet {
                        typ: Type::Request,
                        seq,
                        fun,
                        payload,
                    },
                    &mut buffer,
                )
                .unwrap(),
            )
            .unwrap();
        trace!("sending request: [{}]", payload);
        seq
    }

    fn send_response(&mut self, seq: Sequence, fun: FunctionId, payload: &[u8]) {
        let mut buffer = [0u8; UART_BUFFER_SIZE_BYTES];
        self.uart_writer
            .write(
                postcard::to_slice(
                    &Packet {
                        typ: Type::Response,
                        seq,
                        fun,
                        payload,
                    },
                    &mut buffer,
                )
                .unwrap(),
            )
            .unwrap();
        trace!("sending response: [{}]", payload);
    }

    async fn receive_respone_poll<'a>(&mut self, seq: Sequence) -> &'a [u8] {
        let payload = loop {
            if let Some(response) = cortex_m::interrupt::free(|cs| {
                let packet = PACKET_BUFFER.borrow(cs).borrow_mut()[seq as usize];
                if packet.is_some() {
                    PACKET_BUFFER.borrow(cs).borrow_mut()[seq as usize] = None;
                }
                packet
            }) {
                break response.payload;
            }
            Mono::delay(500.micros()).await;
        };
        trace!("receiving response: [{}]", payload);
        payload
    }
}

pub trait RemoteInvoker {
    async fn invoke<'b, Q, R>(&mut self, fun: u8, request: Q) -> R
    where
        Q: Serialize + Format,
        R: Deserialize<'b> + Format;
}

impl RemoteInvoker for UartSender {
    async fn invoke<'b, Q, R>(&mut self, fun: u8, request: Q) -> R
    where
        Q: Serialize + Format,
        R: Deserialize<'b> + Format,
    {
        let mut request_buffer = [0u8; 256];
        let payload = postcard::to_slice(&request, &mut request_buffer).unwrap();

        let seq = self.send_request(fun, payload);

        let response_buffer = self.receive_respone_poll(seq).await;
        postcard::from_bytes(response_buffer).unwrap()
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Format, PartialEq, Serialize)]
pub enum Type {
    Request,
    Response,
}
pub type FunctionId = u8;
pub type Sequence = u8;

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
pub struct Packet<'a> {
    typ: Type,
    seq: Sequence,
    fun: FunctionId,
    payload: &'a [u8],
}

use alloc::{boxed::Box, vec::Vec};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use defmt::{trace, Format};
use embedded_io::{Read, Write};
use hal::{
    gpio,
    uart::{self, Reader, UartPeripheral, Writer},
};
use rtic_monotonics::{rp2040::prelude::*, Monotonic};
use serde::{Deserialize, Serialize};

use crate::{kb::Mono, ping::PingResponse};

const UART_BUFFER_SIZE_BYTES: usize = 256;

static SOCKET_BUFFER: Mutex<RefCell<[Option<Packet>; SequenceNumber::MAX as usize + 1]>> =
    Mutex::new(RefCell::new([None; SequenceNumber::MAX as usize + 1]));
static FUNCTION_REGISTRY: Mutex<RefCell<[Option<Function>; FunctionId::MAX as usize + 1]>> =
    Mutex::new(RefCell::new([None; FunctionId::MAX as usize + 1]));
static mut INCOMING_PACKET_SEQUENCE_NUMBER: Option<SequenceNumber> = None;

type Function = fn(&[u8]) -> Vec<u8>;

pub struct RemoteExecutor {}

impl RemoteExecutor {
    pub fn new() -> Self {
        RemoteExecutor {}
    }

    pub fn register_function(&mut self, function_id: FunctionId, function: Function) {
        cortex_m::interrupt::free(|cs| {
            FUNCTION_REGISTRY.borrow(cs).borrow_mut()[function_id as usize] = Some(function);
        });
    }

    pub async fn listen(&mut self, client: &RefCell<UartTransport>) {
        let sequence_number = match unsafe { INCOMING_PACKET_SEQUENCE_NUMBER } {
            Some(sequence_number) => sequence_number,
            None => return,
        };

        let packet = match cortex_m::interrupt::free(|cs| {
            SOCKET_BUFFER.borrow(cs).borrow()[sequence_number as usize]
        }) {
            Some(packet) => packet,
            None => return,
        };

        let function = match cortex_m::interrupt::free(|cs| {
            FUNCTION_REGISTRY.borrow(cs).borrow()[packet.function_id as usize]
        }) {
            Some(function) => function,
            None => return,
        };

        let res = function(packet.payload);
        client
            .borrow_mut()
            .send_packet(packet.sequence_number, packet.function_id, res.as_slice());

        unsafe { INCOMING_PACKET_SEQUENCE_NUMBER = None };
    }
}

pub struct UartReceiver {
    uart_reader: Reader<
        hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >,
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
        UartReceiver { uart_reader }
    }

    pub fn read_into_buffer(&mut self) {
        trace!("read_into_buffer()");

        let mut buffer = [0u8; 1024];
        if self.uart_reader.read_raw(&mut buffer).is_err() {
            return;
        }
        let packet: Packet = postcard::from_bytes(&buffer).unwrap();
        trace!("packet: [{}]", packet);

        let payload: PingResponse = postcard::from_bytes(packet.payload).unwrap();
        trace!("packet.payload: [{}]", payload);

        cortex_m::interrupt::free(|cs| {
            SOCKET_BUFFER.borrow(cs).borrow_mut()[packet.sequence_number as usize] = Some(Packet {
                sequence_number: packet.sequence_number,
                function_id: packet.function_id,
                payload: Box::leak(packet.payload.to_vec().into_boxed_slice()),
            });
            unsafe { INCOMING_PACKET_SEQUENCE_NUMBER = Some(packet.sequence_number) };
        });
    }
}

pub struct UartTransport {
    uart_writer: Writer<
        hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >,
}

impl UartTransport {
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
        UartTransport { uart_writer }
    }

    fn get_sequence_number() -> SequenceNumber {
        static mut SEQUENCE_NUMBER: SequenceNumber = 0;
        unsafe {
            SEQUENCE_NUMBER = SEQUENCE_NUMBER.wrapping_add(1);
            SEQUENCE_NUMBER
        }
    }

    fn send_packet(
        &mut self,
        sequence_number: SequenceNumber,
        function_id: FunctionId,
        payload: &[u8],
    ) -> SequenceNumber {
        let mut buffer = [0u8; UART_BUFFER_SIZE_BYTES];
        self.uart_writer
            .write(
                postcard::to_slice(
                    &Packet {
                        sequence_number,
                        function_id,
                        payload,
                    },
                    &mut buffer,
                )
                .unwrap(),
            )
            .unwrap();
        trace!("sending: [{}]", payload);
        sequence_number
    }

    async fn receive_packet<'a>(&mut self, sequence_number: SequenceNumber) -> Packet<'a> {
        loop {
            if let Some(response) = cortex_m::interrupt::free(|cs| {
                let packet = SOCKET_BUFFER.borrow(cs).borrow_mut()[sequence_number as usize];
                if packet.is_some() {
                    SOCKET_BUFFER.borrow(cs).borrow_mut()[sequence_number as usize] = None;
                }
                packet
            }) {
                trace!("receiving: [{}]", response);
                break response;
            }
            Mono::delay(500.millis()).await;
        }
    }
}

pub type FunctionId = u8;
pub type SequenceNumber = u8;

pub trait RemoteInvoker {
    async fn invoke<'a, Q, R>(&mut self, function_id: u8, request: Q) -> R
    where
        Q: Serialize + Format,
        R: Deserialize<'a> + Format;
}

impl RemoteInvoker for UartTransport {
    async fn invoke<'b, Q, R>(&mut self, function_id: u8, request: Q) -> R
    where
        Q: Serialize + Format,
        R: Deserialize<'b> + Format,
    {
        let mut request_buffer = [0u8; 256];
        let payload = postcard::to_slice(&request, &mut request_buffer).unwrap();

        let sequence_number = UartTransport::get_sequence_number();
        self.send_packet(sequence_number, function_id, payload);

        let response_packet = self.receive_packet(sequence_number).await;
        postcard::from_bytes(response_packet.payload).unwrap()
    }
}

pub trait Handler {
    fn handle<'a, Q, R>(&mut self, request: Q) -> R
    where
        Q: Deserialize<'a> + Format,
        R: Serialize + Format;
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Format, Serialize)]
pub struct Packet<'a> {
    // TODO: add packet type (req/res)?
    sequence_number: SequenceNumber,
    function_id: FunctionId,
    payload: &'a [u8],
}

#[derive(Debug, Format)]
enum Err {
    ReadErrorType(#[defmt(Debug2Format)] uart::ReadErrorType),
}

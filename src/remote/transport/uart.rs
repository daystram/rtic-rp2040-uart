use alloc::{borrow::ToOwned, boxed::Box, vec::Vec};
use embedded_io::Write;
use hal::{
    gpio,
    uart::{Reader, Writer},
};
use rtic_monotonics::{rp2040::prelude::*, Monotonic};
use rtic_sync::channel::{Receiver, Sender};
use serde::{Deserialize, Serialize};

use crate::{
    kb::Mono,
    remote::{transport::Sequence, MethodId, RemoteInvoker, ServiceId},
};

use super::{Kind, Packet, TransportReceiver, TransportSender};

const UART_FRAME_BUFFER_SIZE_BYTES: usize = 256;

pub struct UartReceiver {
    uart_reader: Reader<
        hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >,
    seq_sender: Option<Sender<'static, Sequence, { super::super::REQUEST_SEQUENCE_QUEUE_SIZE }>>,
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
    ) -> Receiver<'static, Sequence, { super::super::REQUEST_SEQUENCE_QUEUE_SIZE }> {
        let (seq_sender, seq_receiver) =
            rtic_sync::make_channel!(Sequence, { super::super::REQUEST_SEQUENCE_QUEUE_SIZE });
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
            defmt::error!(
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

        cortex_m::interrupt::free(|cs| {
            if let Some(ref mut old_packet) =
                super::PACKET_BUFFER.borrow(cs).borrow_mut()[packet.sequence as usize]
            {
                // warn!("dropping on wrap: size={}B", mem::size_of_val(old_packet));
                drop(unsafe { Box::from_raw(old_packet.payload as *const [u8] as *mut [u8]) });
            }
            super::PACKET_BUFFER.borrow(cs).borrow_mut()[packet.sequence as usize] = Some(Packet {
                kind: packet.kind,
                sequence: packet.sequence,
                service_id: packet.service_id,
                method_id: packet.method_id,
                payload: Box::leak(packet.payload.to_owned().into_boxed_slice()),
            });
        });

        if packet.kind == super::Kind::Request {
            if let Some(ref mut seq_sender) = self.seq_sender {
                if seq_sender.try_send(packet.sequence).is_err() {
                    defmt::error!(
                        "request sequence queue is full, request dropped: seq={}",
                        packet.sequence
                    );
                };
            }
        }
    }
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
                super::PACKET_BUFFER.borrow(cs).borrow_mut()[seq as usize].take()
            }) {
                break response.payload;
            }
            Mono::delay(50.micros()).await;
        };

        let payload = stored_payload.to_owned();
        drop(unsafe { Box::from_raw(stored_payload as *const [u8] as *mut [u8]) });
        payload
    }
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

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(associated_type_defaults)]
#![feature(trait_alias)]
#![feature(async_closure)]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)]
#![allow(refining_impl_trait)]
#![allow(clippy::type_complexity)]
#![allow(clippy::too_many_arguments)]
mod heartbeat;
mod key;
mod keyboard;
mod matrix;
mod ping;
mod processor;
mod remote;
mod rotary;
mod split;
mod util;

extern crate alloc;
extern crate rp2040_hal as hal;
use {defmt_rtt as _, panic_probe as _};

#[rtic::app(
    device = hal::pac,
    dispatchers = [TIMER_IRQ_1, TIMER_IRQ_2]
)]
mod kb {
    // The linker will place this boot block at the start of our program image.
    // We need this to help the ROM bootloader get our code up and running.
    #[link_section = ".boot2"]
    #[used]
    pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

    #[global_allocator]
    pub static HEAP: embedded_alloc::Heap = embedded_alloc::Heap::empty();
    const HEAP_SIZE_BYTES: usize = 16384; // 16 KB
    static mut HEAP_MEM: [core::mem::MaybeUninit<u8>; HEAP_SIZE_BYTES] =
        [core::mem::MaybeUninit::uninit(); HEAP_SIZE_BYTES];

    use alloc::{boxed::Box, rc::Rc, vec::Vec};
    use core::cell::RefCell;
    use hal::{
        clocks::{init_clocks_and_plls, Clock},
        fugit::RateExtU32,
        gpio, pwm, sio, uart, usb, Sio, Watchdog,
    };
    use rtic_monotonics::{rp2040::prelude::*, Monotonic};
    use rtic_sync::{
        arbiter::Arbiter,
        channel::{Receiver, Sender},
    };
    use split::SideDetector;
    use usb_device::{
        bus::UsbBusAllocator,
        device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
        UsbError,
    };
    use usbd_human_interface_device::{
        device::keyboard::{NKROBootKeyboard, NKROBootKeyboardConfig},
        prelude::{UsbHidClass, UsbHidClassBuilder},
        UsbHidError,
    };

    use crate::{
        heartbeat::HeartbeatLED,
        key::{Action, Edge, Key},
        keyboard,
        matrix::{BasicVerticalSwitchMatrix, SplitScanner, SplitSwitchMatrix},
        ping::Ping,
        processor::{
            events::rgb::FrameIterator,
            input::debounce::KeyMatrixRisingFallingDebounceProcessor,
            mapper::{Input, Mapper},
            Event, EventsProcessor, InputProcessor,
        },
        remote::{
            self,
            transport::{
                uart::{UartReceiver, UartSender},
                Sequence, TransportReceiver,
            },
            Executor,
        },
        rotary::RotaryEncoder,
        split::{self, Mode},
        util,
    };

    rp2040_timer_monotonic!(Mono);

    const INPUT_CHANNEL_BUFFER_SIZE: usize = 1;
    const KEYS_CHANNEL_BUFFER_SIZE: usize = 1;

    const INPUT_SCANNER_TARGET_POLL_FREQ: u64 = 1000;
    const HID_REPORTER_TARGET_POLL_FREQ: u64 = 1000;
    const INPUT_SCANNER_TARGET_POLL_PERIOD_MICROS: u64 =
        1_000_000u64 / INPUT_SCANNER_TARGET_POLL_FREQ;
    const HID_REPORTER_TARGET_POLL_PERIOD_MICROS: u64 =
        1_000_000u64 / HID_REPORTER_TARGET_POLL_FREQ;

    const DEBUG_LOG_INPUT_SCANNER_ENABLE_TIMING: bool = false;
    const DEBUG_LOG_INPUT_SCANNER_INTERVAL: u64 = 50;
    const DEBUG_LOG_PROCESSOR_ENABLE_TIMING: bool = false;
    const DEBUG_LOG_PROCESSOR_INTERVAL: u64 = 50;
    const DEBUG_LOG_EVENTS: bool = true;
    const DEBUG_LOG_SENT_KEYS: bool = false;

    #[shared]
    struct Shared {
        is_usb_connected: bool,
        uart_sender: Arbiter<Rc<RefCell<UartSender>>>,
        usb_device: UsbDevice<'static, usb::UsbBus>,
        usb_keyboard: UsbHidClass<
            'static,
            usb::UsbBus,
            frunk::HList!(NKROBootKeyboard<'static, usb::UsbBus>),
        >,
    }

    #[local]
    struct Local {
        rotary_encoder: Option<RotaryEncoder>,
        heartbeat_led: HeartbeatLED,
        uart_receiver: UartReceiver,
    }

    #[init(local = [usb_allocator: Option<UsbBusAllocator<usb::UsbBus>> = None])]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        defmt::info!("init()");

        // Soft-reset does not release the hardware spinlocks.
        // Release them now to avoid a deadlock after debug or watchdog reset.
        unsafe { sio::spinlock_reset() }

        // Initialize global memory allocator
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE_BYTES) }

        // Set the ARM SLEEPONEXIT bit to go to sleep after handling interrupts
        // See https://developer.arm.com/docs/100737/0100/power-management/sleep-mode/sleep-on-exit-bit
        ctx.core.SCB.set_sleepdeep();

        // Configure watchdog, monotonics, and clock - The default is to generate a 125 MHz system clock
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        Mono::start(ctx.device.TIMER, &ctx.device.RESETS);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Init channels
        let (input_sender, input_receiver) = rtic_sync::make_channel!(Input<{keyboard::KEY_MATRIX_ROW_COUNT}, {keyboard::KEY_MATRIX_COL_COUNT}>, INPUT_CHANNEL_BUFFER_SIZE);
        let (keys_sender, keys_receiver) =
            rtic_sync::make_channel!(Vec<Key>, KEYS_CHANNEL_BUFFER_SIZE);
        let (frame_sender, frame_receiver) = rtic_sync::make_channel!(Box<dyn FrameIterator>, 1);

        // Init GPIO and PWM slices
        let pins = gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            Sio::new(ctx.device.SIO).gpio_bank0,
            &mut ctx.device.RESETS,
        );
        let mut slices = pwm::Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);

        // Init heartbeat LED
        slices.pwm6.set_ph_correct();
        slices.pwm6.enable();
        slices.pwm6.channel_b.output_to(
            pins.gpio29
                .into_push_pull_output_in_state(gpio::PinState::Low),
        );
        let heartbeat_led = HeartbeatLED::new(Box::new(slices.pwm6.channel_b));

        // Init HID device
        defmt::info!("init usb allocator");
        let usb_allocator = ctx
            .local
            .usb_allocator
            .insert(UsbBusAllocator::new(usb::UsbBus::new(
                ctx.device.USBCTRL_REGS,
                ctx.device.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut ctx.device.RESETS,
            )));

        defmt::info!("init usb keyboard");
        let usb_keyboard = UsbHidClassBuilder::new()
            .add_device(NKROBootKeyboardConfig::default())
            .build(usb_allocator);

        defmt::info!("init usb device");
        let usb_device = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x1111, 0x1111))
            .strings(&[StringDescriptors::default()
                .manufacturer("daystram")
                .product("kb")
                .serial_number("8888")])
            .unwrap()
            .build();

        // Init transport and remote invoker
        let mut uart_peripheral = uart::UartPeripheral::new(
            ctx.device.UART0,
            (pins.gpio0.into_function(), pins.gpio1.into_function()),
            &mut ctx.device.RESETS,
        )
        .enable(
            uart::UartConfig::new(
                // 115_200.Hz(),
                230_400.Hz(),
                uart::DataBits::Eight,
                Some(uart::Parity::Even),
                uart::StopBits::One,
            ),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
        uart_peripheral.set_fifos(true);
        let (uart_reader, uart_writer) = uart_peripheral.split();
        let uart_sender = Arbiter::new(Rc::new(RefCell::new(UartSender::new(uart_writer))));
        let mut uart_receiver = UartReceiver::new(uart_reader);
        let seq_sender = uart_receiver.initialize_seq_sender();

        // Detect side
        SideDetector::new(Box::new(pins.gpio2.into_pull_down_input())).detect();

        // Init keyboard
        let key_matrix = Some(SplitSwitchMatrix::new(BasicVerticalSwitchMatrix::new(
            [
                Box::new(pins.gpio16.into_pull_down_input()),
                Box::new(pins.gpio15.into_pull_down_input()),
            ],
            [Box::new(pins.gpio14.into_push_pull_output())],
        )));
        let rotary_encoder = None;
        let ping = None;

        // Begin
        start_wait_usb::spawn(
            1.secs(),
            input_sender,
            input_receiver,
            keys_sender,
            keys_receiver,
            frame_sender,
            frame_receiver,
            seq_sender,
            ping,
            key_matrix,
        )
        .ok();

        defmt::info!("init() done");
        (
            Shared {
                is_usb_connected: false,
                uart_sender,
                usb_device,
                usb_keyboard,
            },
            Local {
                rotary_encoder,
                heartbeat_led,
                uart_receiver,
            },
        )
    }

    #[idle()]
    fn idle(_ctx: idle::Context) -> ! {
        defmt::info!("idle()");
        loop {
            // https://developer.arm.com/documentation/ddi0406/c/Application-Level-Architecture/Instruction-Details/Alphabetical-list-of-instructions/WFI
            rtic::export::wfi()
        }
    }

    // ============================= Master and Slave
    #[task(shared=[is_usb_connected], priority = 1)]
    async fn start_wait_usb(
        mut ctx: start_wait_usb::Context,
        timeout: <Mono as Monotonic>::Duration,
        input_sender: Sender<
            'static,
            Input<{ keyboard::KEY_MATRIX_ROW_COUNT }, { keyboard::KEY_MATRIX_COL_COUNT }>,
            INPUT_CHANNEL_BUFFER_SIZE,
        >,
        input_receiver: Receiver<
            'static,
            Input<{ keyboard::KEY_MATRIX_ROW_COUNT }, { keyboard::KEY_MATRIX_COL_COUNT }>,
            INPUT_CHANNEL_BUFFER_SIZE,
        >,
        keys_sender: Sender<'static, Vec<Key>, KEYS_CHANNEL_BUFFER_SIZE>,
        keys_receiver: Receiver<'static, Vec<Key>, KEYS_CHANNEL_BUFFER_SIZE>,
        frame_sender: Sender<'static, Box<dyn FrameIterator>, 1>,
        _frame_receiver: Receiver<'static, Box<dyn FrameIterator>, 1>,
        seq_sender: Receiver<'static, Sequence, { remote::REQUEST_SEQUENCE_QUEUE_SIZE }>,
        ping: Option<Ping>,
        key_matrix: Option<
            SplitSwitchMatrix<
                { keyboard::KEY_MATRIX_ROW_COUNT },
                { keyboard::KEY_MATRIX_COL_COUNT },
            >,
        >,
    ) {
        defmt::info!("start_wait_usb()");

        // Start USB tasks
        hid_usb_tick::spawn().ok();
        hid_reporter::spawn(keys_receiver).ok();
        unsafe { hal::pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ) }

        Mono::delay(timeout).await;

        split::set_self_mode(ctx.shared.is_usb_connected.lock(|is_usb_connected| {
            if *is_usb_connected {
                Mode::Master
            } else {
                Mode::Slave
            }
        }));
        defmt::warn!(
            "detected as {} {}",
            split::get_self_mode(),
            split::get_self_side()
        );

        match split::get_self_mode() {
            Mode::Master => {
                heartbeat::spawn(20.millis()).ok();
                if let Some(ping) = ping {
                    master_ping::spawn(ping).ok();
                }
                master_input_scanner::spawn(key_matrix, input_sender).ok();
                master_processor::spawn(input_receiver, keys_sender, frame_sender).ok();
            }
            Mode::Slave => {
                // Initialize remote executor and register services
                let mut remote_executor = Executor::new(seq_sender);
                if let Some(ping) = ping {
                    remote_executor.register_service(Box::new(ping)).await;
                }
                if let Some(key_matrix) = key_matrix {
                    remote_executor.register_service(Box::new(key_matrix)).await;
                }

                heartbeat::spawn(100.millis()).ok();
                slave_server::spawn(remote_executor).ok();
            }
        }
        unsafe { hal::pac::NVIC::unmask(hal::pac::Interrupt::UART0_IRQ) }
    }

    #[task(local=[heartbeat_led], priority = 2)]
    async fn heartbeat(ctx: heartbeat::Context, period: <Mono as Monotonic>::Duration) {
        defmt::info!("heartbeat()");
        ctx.local.heartbeat_led.cycle(period).await
    }

    #[task(binds = UART0_IRQ, local = [uart_receiver], priority = 1)]
    fn receive_uart(ctx: receive_uart::Context) {
        // defmt::warn!("UART0_IRQ start");
        // let start_time = Mono::now();
        ctx.local.uart_receiver.read_into_buffer();
        // let end_time = Mono::now();
        // util::log_duration(
        //     util::LogDurationTag::UARTIRQRecieveBuffer,
        //     start_time,
        //     end_time,
        // );
        // defmt::warn!("UART0_IRQ done");
    }
    // ============================= Master and Slave

    // ============================= Slave
    #[task (shared=[&uart_sender], priority = 1)]
    async fn slave_server(ctx: slave_server::Context, mut remote_executor: Executor) {
        defmt::info!("slave_server()");
        remote_executor.listen(ctx.shared.uart_sender).await;
    }
    // ============================= Slave

    // ============================= Master
    #[task (shared=[&uart_sender], priority = 1)]
    async fn master_ping(ctx: master_ping::Context, mut ping: Ping) {
        defmt::info!("master_ping()");
        let mut counter = 0u8;
        loop {
            counter = (counter + 1) % 10;
            if counter < 5 {
                ping.ping_a(ctx.shared.uart_sender).await
            } else {
                ping.ping_b(ctx.shared.uart_sender).await
            }
            Mono::delay(10.millis()).await;
        }
    }

    #[task (shared=[&uart_sender], local=[rotary_encoder], priority = 1)]
    async fn master_input_scanner(
        ctx: master_input_scanner::Context,
        mut key_matrix: Option<
            SplitSwitchMatrix<
                { keyboard::KEY_MATRIX_ROW_COUNT },
                { keyboard::KEY_MATRIX_COL_COUNT },
            >,
        >,
        mut input_sender: Sender<
            'static,
            Input<{ keyboard::KEY_MATRIX_ROW_COUNT }, { keyboard::KEY_MATRIX_COL_COUNT }>,
            INPUT_CHANNEL_BUFFER_SIZE,
        >,
    ) {
        defmt::info!("master_input_scanner()");
        let mut poll_end_time = Mono::now();
        let mut n: u64 = 0;
        loop {
            let scan_start_time = Mono::now();

            let uart_sender = ctx.shared.uart_sender;
            let key_matrix_result = match key_matrix {
                Some(ref mut key_matrix) => key_matrix.scan(uart_sender).await,
                None => Default::default(),
            };
            let rotary_encoder_result = match ctx.local.rotary_encoder {
                Some(rotary_encoder) => rotary_encoder.scan(),
                None => Default::default(),
            };

            input_sender
                .try_send(Input {
                    key_matrix_result,
                    rotary_encoder_result,
                })
                .ok(); // drop data if buffer is full

            if DEBUG_LOG_INPUT_SCANNER_ENABLE_TIMING && n % DEBUG_LOG_INPUT_SCANNER_INTERVAL == 0 {
                let scan_end_time = Mono::now();
                defmt::debug!(
                    "[{}] input_scanner: {} us\tpoll: {} us\trate: {} Hz\t budget: {} %",
                    n,
                    (scan_end_time - scan_start_time).to_micros(),
                    (scan_end_time - poll_end_time).to_micros(),
                    1_000_000u64 / (scan_end_time - poll_end_time).to_micros(),
                    (scan_end_time - scan_start_time).to_micros() * 100
                        / INPUT_SCANNER_TARGET_POLL_PERIOD_MICROS
                );
            }

            poll_end_time = Mono::now();
            util::log_duration(
                util::LogDurationTag::ClientLatency,
                scan_start_time,
                poll_end_time,
            );
            util::log_heap();

            n = n.wrapping_add(1);
            Mono::delay_until(scan_start_time + INPUT_SCANNER_TARGET_POLL_PERIOD_MICROS.micros())
                .await;
        }
    }

    #[task(priority = 2)]
    async fn master_processor(
        _: master_processor::Context,
        mut input_receiver: Receiver<
            'static,
            Input<{ keyboard::KEY_MATRIX_ROW_COUNT }, { keyboard::KEY_MATRIX_COL_COUNT }>,
            INPUT_CHANNEL_BUFFER_SIZE,
        >,
        mut keys_sender: Sender<'static, Vec<Key>, KEYS_CHANNEL_BUFFER_SIZE>,
        _frame_sender: Sender<'static, Box<dyn FrameIterator>, 1>,
    ) {
        defmt::info!("master_processor()");
        let input_processors: &mut [&mut dyn InputProcessor<
            { keyboard::KEY_MATRIX_ROW_COUNT },
            { keyboard::KEY_MATRIX_COL_COUNT },
        >] = &mut [&mut KeyMatrixRisingFallingDebounceProcessor::new(
            10.millis(),
        )];
        let mut mapper = Mapper::new(keyboard::get_input_map());
        let events_processors: &mut [&mut dyn EventsProcessor<keyboard::Layer>] = &mut [];

        let mut poll_end_time = Mono::now();
        let mut n: u64 = 0;
        while let Ok(mut input) = input_receiver.recv().await {
            let process_start_time = Mono::now();
            if input_processors
                .iter_mut()
                .try_for_each(|p| p.process(&mut input))
                .is_err()
            {
                continue;
            }

            let mut events = Vec::<Event<keyboard::Layer>>::with_capacity(10);
            mapper.map(&input, &mut events);

            if DEBUG_LOG_EVENTS {
                events
                    .iter()
                    .filter(|e| e.edge != Edge::None)
                    .for_each(|e| {
                        defmt::debug!("[{}] event: action: {} edge: {}", n, e.action, e.edge)
                    });
            }

            if events_processors
                .iter_mut()
                .try_for_each(|p| p.process(&mut events))
                .is_err()
            {
                continue;
            }

            keys_sender
                .try_send(
                    events
                        .into_iter()
                        .filter_map(|e| match e.action {
                            Action::Key(k) => Some(k),
                            _ => None,
                        })
                        .collect(),
                )
                .ok(); // drop data if buffer is full

            if DEBUG_LOG_PROCESSOR_ENABLE_TIMING && (n % DEBUG_LOG_PROCESSOR_INTERVAL == 0) {
                let scan_end_time = Mono::now();
                defmt::debug!(
                    "[{}] processor: {} us\tpoll: {} us\trate: {} Hz\t budget: {} %",
                    n,
                    (scan_end_time - process_start_time).to_micros(),
                    (scan_end_time - poll_end_time).to_micros(),
                    1_000_000u64 / (scan_end_time - poll_end_time).to_micros(),
                    (scan_end_time - process_start_time).to_micros() * 100
                        / INPUT_SCANNER_TARGET_POLL_PERIOD_MICROS
                );
            }

            poll_end_time = Mono::now();
            n = n.wrapping_add(1);
        }
    }

    #[task(shared=[usb_keyboard], priority = 1)]
    async fn hid_reporter(
        mut ctx: hid_reporter::Context,
        mut keys_receiver: Receiver<'static, Vec<Key>, KEYS_CHANNEL_BUFFER_SIZE>,
    ) {
        defmt::info!("hid_reporter()");
        while let Ok(keys) = keys_receiver.recv().await {
            let start_time = Mono::now();
            if DEBUG_LOG_SENT_KEYS {
                defmt::debug!("keys: {:?}", keys.as_slice());
            }

            ctx.shared.usb_keyboard.lock(|k| {
                match k.device().write_report(keys.into_iter().map(|k| k.into())) {
                    Ok(_) => {}
                    Err(UsbHidError::WouldBlock) => {}
                    Err(UsbHidError::Duplicate) => {}
                    Err(e) => {
                        core::panic!("Failed to write keyboard report: {:?}", e);
                    }
                }
            });

            Mono::delay_until(start_time + HID_REPORTER_TARGET_POLL_PERIOD_MICROS.micros()).await;
        }
    }

    #[task(binds = USBCTRL_IRQ, shared = [usb_device, usb_keyboard, is_usb_connected], priority = 1)]
    fn hid_reader(ctx: hid_reader::Context) {
        (ctx.shared.usb_device, ctx.shared.usb_keyboard, ctx.shared.is_usb_connected).lock(|usb_device, usb_keyboard, is_usb_connected| {
            if usb_device.poll(&mut [usb_keyboard]) {
                *is_usb_connected = true; // usb connection detected
                match usb_keyboard.device().read_report() {
                    Ok(leds) => {
                        defmt::debug!(
                            "\nnum_lock: {}\ncaps_lock: {}\nscroll_lock: {}\ncompose: {}\nkana: {}\n",
                            leds.num_lock,
                            leds.caps_lock,
                            leds.scroll_lock,
                            leds.compose,
                            leds.kana,
                        );
                    }
                    Err(UsbError::WouldBlock) => {}
                    Err(e) => {
                        core::panic!("Failed to read keyboard report: {:?}", e)
                    }
                }
            }
        });
    }

    #[task(
        shared = [usb_keyboard],
        priority = 1,
    )]
    async fn hid_usb_tick(mut ctx: hid_usb_tick::Context) {
        defmt::info!("hid_usb_tick()");
        loop {
            ctx.shared.usb_keyboard.lock(|k| match k.tick() {
                Ok(_) => {}
                Err(UsbHidError::WouldBlock) => {}
                Err(e) => {
                    core::panic!("Failed to process keyboard tick: {:?}", e)
                }
            });
            Mono::delay(1.millis()).await;
        }
    }
    // ============================= Master
}

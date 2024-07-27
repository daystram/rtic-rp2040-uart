#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(associated_type_defaults)]
#![feature(trait_alias)]
#![allow(clippy::type_complexity)]
mod heartbeat;
mod util;

extern crate alloc;
extern crate rp2040_hal as hal;
use {
    defmt::Format,
    defmt_rtt as _, panic_probe as _,
    serde::{Deserialize, Serialize},
};

#[rtic::app(
    device = hal::pac,
    dispatchers = [TIMER_IRQ_1, TIMER_IRQ_2]
)]
mod rtic_rp2040_uart {
    // The linker will place this boot block at the start of our program image.
    // We need this to help the ROM bootloader get our code up and running.
    #[link_section = ".boot2"]
    #[used]
    pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

    #[global_allocator]
    static HEAP: embedded_alloc::Heap = embedded_alloc::Heap::empty();
    const HEAP_SIZE_BYTES: usize = 16384; // 16 KB
    static mut HEAP_MEM: [core::mem::MaybeUninit<u8>; HEAP_SIZE_BYTES] =
        [core::mem::MaybeUninit::uninit(); HEAP_SIZE_BYTES];

    use alloc::boxed::Box;
    use defmt::{debug, info};
    use embedded_io::{Read, Write};
    use hal::{
        clocks::{init_clocks_and_plls, Clock},
        fugit::RateExtU32,
        gpio, pwm, sio, uart, usb, Sio, Watchdog,
    };
    use rtic_monotonics::{rp2040::prelude::*, Monotonic};
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
        heartbeat::HeartbeatLED, KeyMatrixScanRequest, KeyMatrixScanResponse, Message, Mode,
        Payload,
    };

    rp2040_timer_monotonic!(Mono);

    const UART_BUFFER_SIZE_BYTES: usize = core::mem::size_of::<Message>();

    #[shared]
    struct Shared {
        is_usb_connected: bool,
        uart_writer: uart::Writer<
            hal::pac::UART0,
            (
                gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
                gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
            ),
        >,
        usb_device: UsbDevice<'static, usb::UsbBus>,
        usb_keyboard: UsbHidClass<
            'static,
            usb::UsbBus,
            frunk::HList!(NKROBootKeyboard<'static, usb::UsbBus>),
        >,
    }

    #[local]
    struct Local {
        heartbeat_led: HeartbeatLED,
        uart_reader: uart::Reader<
            hal::pac::UART0,
            (
                gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
                gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
            ),
        >,
    }

    #[init(local = [usb_allocator: Option<UsbBusAllocator<usb::UsbBus>> = None])]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        info!("init()");

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
        info!("init usb allocator");
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

        info!("init usb keyboard");
        let usb_keyboard = UsbHidClassBuilder::new()
            .add_device(NKROBootKeyboardConfig::default())
            .build(usb_allocator);

        info!("init usb device");
        let usb_device = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x1111, 0x1111))
            .strings(&[StringDescriptors::default()
                .manufacturer("daystram")
                .product("kb")
                .serial_number("8888")])
            .unwrap()
            .build();

        // Init UART
        let uart_device = uart::UartPeripheral::new(
            ctx.device.UART0,
            (pins.gpio0.into_function(), pins.gpio1.into_function()),
            &mut ctx.device.RESETS,
        )
        .enable(
            uart::UartConfig::new(9600.Hz(), uart::DataBits::Eight, None, uart::StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

        let (mut uart_reader, uart_writer) = uart_device.split();
        uart_reader.enable_rx_interrupt();

        hid_usb_tick::spawn().ok();
        begin_wait_usb::spawn(500.millis()).ok();

        info!("init() done");
        (
            Shared {
                is_usb_connected: false,
                uart_writer,
                usb_device,
                usb_keyboard,
            },
            Local {
                heartbeat_led,
                uart_reader,
            },
        )
    }

    #[idle()]
    fn idle(_ctx: idle::Context) -> ! {
        info!("idle()");
        loop {
            // https://developer.arm.com/documentation/ddi0406/c/Application-Level-Architecture/Instruction-Details/Alphabetical-list-of-instructions/WFI
            rtic::export::wfi()
        }
    }

    #[task(shared=[is_usb_connected], priority = 1)]
    async fn begin_wait_usb(
        mut ctx: begin_wait_usb::Context,
        timeout: <Mono as Monotonic>::Duration,
    ) {
        info!("begin_wait_usb()");
        unsafe { hal::pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ) }

        Mono::delay(timeout).await;

        ctx.shared.is_usb_connected.lock(|is_usb_connected| {
            begin::spawn(if *is_usb_connected {
                Mode::Master
            } else {
                Mode::Slave
            })
            .ok()
        });
    }

    #[task(priority = 1)]
    async fn begin(_: begin::Context, mode: Mode) {
        info!("begin()");
        heartbeat::spawn(match mode {
            Mode::Master => 50.millis(),
            Mode::Slave => 500.millis(),
        })
        .ok();
        match mode {
            Mode::Master => master::spawn().ok(),
            Mode::Slave => slave::spawn().ok(),
        };
        unsafe { hal::pac::NVIC::unmask(hal::pac::Interrupt::UART0_IRQ) }
    }

    #[task(local=[heartbeat_led], priority = 2)]
    async fn heartbeat(ctx: heartbeat::Context, period: <Mono as Monotonic>::Duration) {
        info!("heartbeat()");
        ctx.local.heartbeat_led.cycle(period).await
    }

    #[task(shared=[uart_writer], priority = 1)]
    async fn master(mut ctx: master::Context) {
        info!("master()");

        let mut counter: u32 = 0;
        loop {
            let message = Message {
                counter,
                payload: if counter % 10 < 3 {
                    Payload::None
                } else if counter % 10 < 6 {
                    Payload::KeyMatrixScanRequest(KeyMatrixScanRequest {})
                } else {
                    Payload::KeyMatrixScanResponse(KeyMatrixScanResponse { pressed: true })
                },
            };

            debug!("sending: [{}]", message);

            ctx.shared.uart_writer.lock(|uart_writer| {
                let mut buf = [0u8; UART_BUFFER_SIZE_BYTES];
                uart_writer
                    .write(postcard::to_slice(&message, &mut buf).unwrap())
                    .unwrap();
            });

            Mono::delay(500.millis()).await;
            counter = counter.wrapping_add(1);
        }
    }

    #[task(shared=[uart_writer], priority = 1)]
    async fn slave(_: slave::Context) {
        info!("slave()");
    }

    #[task(binds = UART0_IRQ, local = [uart_reader], priority = 1)]
    fn receive_uart(ctx: receive_uart::Context) {
        let mut buf = [0; UART_BUFFER_SIZE_BYTES];
        if ctx.local.uart_reader.read(&mut buf).is_err() {
            return;
        }

        let message: Message = postcard::from_bytes(&buf).unwrap();
        debug!("receiving: [{}]", message);
    }

    #[task(binds = USBCTRL_IRQ, shared = [usb_device, usb_keyboard, is_usb_connected], priority = 1)]
    fn hid_reader(ctx: hid_reader::Context) {
        (ctx.shared.usb_device, ctx.shared.usb_keyboard, ctx.shared.is_usb_connected).lock(|usb_device, usb_keyboard, is_usb_connected| {
            if usb_device.poll(&mut [usb_keyboard]) {
                *is_usb_connected = true; // usb connection detected
                match usb_keyboard.device().read_report() {
                    Ok(leds) => {
                        debug!(
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
        info!("hid_usb_tick()");
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
}

#[derive(Clone, Copy, Debug, Format, PartialEq, PartialOrd)]
enum Mode {
    Master,
    Slave,
}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
struct Message {
    counter: u32,
    payload: Payload,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Format, Serialize)]
enum Payload {
    #[default]
    None,
    PingRequest(PingRequest),
    PingResponse(PingResponse),
    KeyMatrixScanRequest(KeyMatrixScanRequest),
    KeyMatrixScanResponse(KeyMatrixScanResponse),
}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
struct PingRequest {}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
struct PingResponse {}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
struct KeyMatrixScanRequest {}

#[derive(Clone, Copy, Debug, Deserialize, Format, Serialize)]
struct KeyMatrixScanResponse {
    pressed: bool,
}

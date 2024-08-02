#![allow(dead_code)]
use alloc::boxed::Box;
use defmt::{debug, error, trace, Format};
use embedded_hal::{digital::OutputPin, pwm::SetDutyCycle};
use hal::gpio;
use rtic_monotonics::rp2040::prelude::*;

use crate::kb::{Mono, HEAP};

pub async fn blink<P: OutputPin>(
    pin: &mut P,
    duty: u8,
    delay: <Mono as Monotonic>::Duration,
) -> Result<(), P::Error> {
    match pin.set_high() {
        Ok(_) => (),
        Err(e) => return Err(e),
    };
    Mono::delay((delay.to_nanos() * duty as u64 / u8::MAX as u64).nanos()).await;
    match pin.set_low() {
        Ok(_) => (),
        Err(e) => return Err(e),
    };
    Mono::delay((delay.to_nanos() * (u8::MAX - duty) as u64 / u8::MAX as u64).nanos()).await;
    Ok(())
}

pub async fn lerp(
    pin: &mut Box<dyn SetDutyCycle<Error = gpio::Error>>,
    from: u16,
    to: u16,
    step: u16,
    delay: <Mono as Monotonic>::Duration,
) {
    let diff = if from < to {
        (to - from) / step
    } else {
        (from - to) / step
    };
    let step_delay = (delay.to_nanos() / step as u64).nanos();

    for d in (0..step)
        .map(|x| x * diff)
        .map(|x| if from < to { from + x } else { from - x })
    {
        pin.set_duty_cycle(d).unwrap();
        Mono::delay(step_delay).await;
    }
}

const ENABLE_LOG_HEAP: bool = true;
const ENABLE_LOG_DURATION: bool = true;
const LOG_HEAP_RATE: u32 = 1000;
const LOG_DURATION_RATE: u32 = 100;

static mut LOG_HEAP_COUNTER: u32 = 0;

pub fn log_heap() {
    if !ENABLE_LOG_HEAP {
        return;
    }
    let counter = unsafe {
        LOG_HEAP_COUNTER = LOG_HEAP_COUNTER.wrapping_add(1);
        LOG_HEAP_COUNTER
    };
    if counter % LOG_HEAP_RATE == 0 {
        error!(
            "[{}] ========= heap stat: free={}B used={}B",
            counter,
            HEAP.free(),
            HEAP.used()
        );
    }
}

#[derive(Clone, Copy, Debug, Format)]
#[repr(u8)]
pub enum LogDurationTag {
    ClientLatency,
    ServerLatency,
    UARTIRQRecieveBuffer,
}

static mut LOG_DURATION_COUNTER: [u32; u8::MAX as usize] = [0; u8::MAX as usize];

pub fn log_duration(
    tag: LogDurationTag,
    start_time: <Mono as Monotonic>::Instant,
    end_time: <Mono as Monotonic>::Instant,
) {
    if !ENABLE_LOG_DURATION {
        return;
    }
    let counter = unsafe {
        LOG_DURATION_COUNTER[tag as usize] = LOG_DURATION_COUNTER[tag as usize].wrapping_add(1);
        LOG_DURATION_COUNTER[tag as usize]
    };
    if counter % LOG_DURATION_RATE == 0 {
        trace!(
            "[{}] ========= {}: {}us",
            counter,
            tag,
            (end_time - start_time).to_micros()
        );
    }
}

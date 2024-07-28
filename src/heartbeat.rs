use alloc::boxed::Box;
use embedded_hal::pwm::SetDutyCycle;
use hal::gpio;
use rtic_monotonics::Monotonic;

use crate::{kb::Mono, util};

const MAX_PWM_POWER: u16 = 0x6000;
const STEP: u16 = 1;

pub struct HeartbeatLED {
    pin: Box<dyn SetDutyCycle<Error = gpio::Error>>,
}

impl HeartbeatLED {
    pub fn new(pin: Box<dyn SetDutyCycle<Error = gpio::Error>>) -> Self {
        HeartbeatLED { pin }
    }

    pub async fn cycle(&mut self, period: <Mono as Monotonic>::Duration) -> ! {
        loop {
            util::lerp(&mut self.pin, 0, MAX_PWM_POWER, STEP, period).await;
            util::lerp(&mut self.pin, MAX_PWM_POWER, 0, STEP, period).await;
        }
    }
}

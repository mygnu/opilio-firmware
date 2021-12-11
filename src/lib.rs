#![no_std]

pub mod controller;
pub mod tacho;

use core::sync::atomic::{AtomicUsize, Ordering};

use controller::FanId;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::{
    gpio::{
        gpioa::{PA0, PA1, PA2, PA3, PA6, PA7},
        gpiob::{PB0, PB1, PB12, PB13, PB14, PB6, PB7},
        Alternate, Floating, Input, Output, PushPull,
    },
    pac,
    pwm::{self, Pwm},
    pwm_input::PwmInput,
    timer::{Tim2NoRemap, Tim3NoRemap, Tim4NoRemap},
};

// global logger
use defmt_rtt as _;
use panic_probe as _;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is
// invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

/// Result type used by Opilio.
pub type Result<T> = ::core::result::Result<T, Error>;

pub enum Error {
    Deserialize,
}

pub type PwmInputTimer = PwmInput<
    pac::TIM4,
    Tim4NoRemap,
    (PB6<Input<Floating>>, PB7<Input<Floating>>),
>;

pub type PwmTimer2 = Pwm<
    pac::TIM2,
    Tim2NoRemap,
    (pwm::C1, pwm::C2, pwm::C3, pwm::C4),
    (
        PA0<Alternate<PushPull>>,
        PA1<Alternate<PushPull>>,
        PA2<Alternate<PushPull>>,
        PA3<Alternate<PushPull>>,
    ),
>;

pub type PwmTimer3 = Pwm<
    pac::TIM3,
    Tim3NoRemap,
    (pwm::C1, pwm::C2, pwm::C3, pwm::C4),
    (
        PA6<Alternate<PushPull>>,
        PA7<Alternate<PushPull>>,
        PB0<Alternate<PushPull>>,
        PB1<Alternate<PushPull>>,
    ),
>;

pub struct MuxController {
    s0: PB12<Output<PushPull>>,
    s1: PB13<Output<PushPull>>,
    s2: PB14<Output<PushPull>>,
}

impl MuxController {
    pub fn new(
        s0: PB12<Output<PushPull>>,
        s1: PB13<Output<PushPull>>,
        s2: PB14<Output<PushPull>>,
    ) -> Self {
        Self { s0, s1, s2 }
    }

    pub fn enable(&mut self, input: FanId) {
        use FanId::*;
        match input {
            F0 => {
                self.s0.set_low().ok();
                self.s1.set_low().ok();
                self.s2.set_low().ok();
            }
            F1 => {
                self.s0.set_high().ok();
                self.s1.set_low().ok();
                self.s2.set_low().ok();
            }
            F2 => {
                self.s0.set_low().ok();
                self.s1.set_high().ok();
                self.s2.set_low().ok();
            }
            F3 => {
                self.s0.set_high().ok();
                self.s1.set_high().ok();
                self.s2.set_low().ok();
            }
            F4 => {
                self.s0.set_low().ok();
                self.s1.set_low().ok();
                self.s2.set_high().ok();
            }
            F5 => {
                self.s0.set_high().ok();
                self.s1.set_low().ok();
                self.s2.set_high().ok();
            }
            F6 => {
                self.s0.set_low().ok();
                self.s1.set_high().ok();
                self.s2.set_high().ok();
            }
            F7 => {
                self.s0.set_high().ok();
                self.s1.set_high().ok();
                self.s2.set_high().ok();
            }
        };
    }
}

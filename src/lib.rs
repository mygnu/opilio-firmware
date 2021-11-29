#![no_std]

pub mod consts;
pub mod controller;

use core::sync::atomic::{AtomicUsize, Ordering};

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

pub enum Error {
    InvalidConfiguration = 0x01,
    ReadingFlash = 0x02,
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

#[derive(Copy, Clone)]
pub enum MuxInput {
    L0,
    L1,
    L2,
    L3,
    L4,
    L5,
    L6,
    L7,
}

impl MuxController {
    pub fn new(
        s0: PB12<Output<PushPull>>,
        s1: PB13<Output<PushPull>>,
        s2: PB14<Output<PushPull>>,
    ) -> Self {
        Self { s0, s1, s2 }
    }

    pub fn enable(&mut self, input: MuxInput) {
        use self::MuxInput::*;
        match input {
            L0 => {
                self.s0.set_low().ok();
                self.s1.set_low().ok();
                self.s2.set_low().ok();
            }
            L1 => {
                self.s0.set_high().ok();
                self.s1.set_low().ok();
                self.s2.set_low().ok();
            }
            L2 => {
                self.s0.set_low().ok();
                self.s1.set_high().ok();
                self.s2.set_low().ok();
            }
            L3 => {
                self.s0.set_high().ok();
                self.s1.set_high().ok();
                self.s2.set_low().ok();
            }
            L4 => {
                self.s0.set_low().ok();
                self.s1.set_low().ok();
                self.s2.set_high().ok();
            }
            L5 => {
                self.s0.set_high().ok();
                self.s1.set_low().ok();
                self.s2.set_high().ok();
            }
            L6 => {
                self.s0.set_low().ok();
                self.s1.set_high().ok();
                self.s2.set_high().ok();
            }
            L7 => {
                self.s0.set_high().ok();
                self.s1.set_high().ok();
                self.s2.set_high().ok();
            }
            _ => {}
        };
    }
}

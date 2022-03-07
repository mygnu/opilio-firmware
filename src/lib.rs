#![no_std]

pub mod controller;
pub mod tacho;
use core::ops::Deref;

use core::sync::atomic::{AtomicUsize, Ordering};

use controller::FanId;
// global logger
use defmt_rtt as _;
use panic_probe as _;
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

use controller::{
    default_configs, Config, BUF_SIZE, CONFIG_SIZE, FLASH_START_OFFSET,
};
use defmt::debug;
use heapless::Vec;
use postcard::{from_bytes, to_vec};
use stm32f1xx_hal::flash::{FlashWriter, SZ_1K};

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

    pub fn switch(&mut self, input: FanId) {
        use FanId::*;
        match input {
            F7 => {
                // I0
                self.s2.set_low();
                self.s1.set_low();
                self.s0.set_low();
            }
            F6 => {
                // I1
                self.s2.set_low();
                self.s1.set_low();
                self.s0.set_high();
            }
            F5 => {
                // I2
                self.s2.set_low();
                self.s1.set_high();
                self.s0.set_low();
            }
            F4 => {
                // I3
                self.s2.set_low();
                self.s1.set_high();
                self.s0.set_high();
            }
            F3 => {
                // I4
                self.s2.set_high();
                self.s1.set_low();
                self.s0.set_low();
            }
            F2 => {
                // I5
                self.s2.set_high();
                self.s1.set_low();
                self.s0.set_high();
            }
            F1 => {
                // I6
                self.s2.set_high();
                self.s1.set_high();
                self.s0.set_low();
            }
            F0 => {
                // I7
                self.s2.set_high();
                self.s1.set_high();
                self.s0.set_high();
            }
        };
    }
}

pub fn load_config_from_flash(writer: &FlashWriter) -> Vec<Config, 8> {
    let mut configs = default_configs();

    let bytes = match writer.read(FLASH_START_OFFSET, SZ_1K as usize) {
        Ok(it) => it,
        _ => return configs,
    };
    for c in configs.iter_mut() {
        let range = c.offset_range();
        let chunk = &bytes[c.offset_range()];
        debug!("range {}; chunk: {}", range, chunk);
        if let Ok(config) = from_bytes::<Config>(chunk) {
            debug!("From memory {}", config);
            if config.is_valid() {
                *c = config;
            }
        }
    }

    configs
}

pub fn save_config_to_flash(
    writer: &mut FlashWriter,
    configs: &Vec<Config, 8>,
) {
    let mut buff: Vec<u8, BUF_SIZE> = Vec::new();

    configs.iter().for_each(|f| {
        let mut ser: Vec<u8, CONFIG_SIZE> = to_vec(f).unwrap();
        debug!("ser {}", ser.deref());
        ser.resize_default(CONFIG_SIZE).unwrap();
        debug!("after resize {}", ser.deref());

        buff.extend(ser.into_iter());

        debug!("buf {}", buff.deref());
    });

    writer.page_erase(FLASH_START_OFFSET).ok();

    writer.write(FLASH_START_OFFSET, buff.deref()).ok();
}

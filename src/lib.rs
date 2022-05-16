#![no_std]

pub mod controller;
pub mod serial_handler;
pub mod tacho;
use core::{
    ops::Deref,
    sync::atomic::{AtomicUsize, Ordering},
};

use controller::{Config, FanId, BUF_SIZE, CONFIG_SIZE, FLASH_START_OFFSET};
use defmt::debug;
// global logger
use defmt_rtt as _;
use heapless::Vec;
use panic_probe as _;
use postcard::{from_bytes, to_vec};
use stm32f1xx_hal::{
    flash::{self, FlashSize, SectorSize, SZ_1K},
    gpio::{
        gpioa::{PA0, PA1, PA2, PA3, PA6, PA7},
        gpiob::{PB0, PB1, PB12, PB13, PB14, PB6, PB7},
        Alternate, Floating, Input, Output, PushPull,
    },
    pac,
    timer::{
        Ch, PwmHz, PwmInput, Tim2NoRemap, Tim3NoRemap, Tim4NoRemap, C1, C2, C3,
        C4,
    },
};

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

pub type PwmTimer2 = PwmHz<
    pac::TIM2,
    Tim2NoRemap,
    (Ch<C1>, Ch<C2>, Ch<C3>, Ch<C4>),
    (
        PA0<Alternate<PushPull>>,
        PA1<Alternate<PushPull>>,
        PA2<Alternate<PushPull>>,
        PA3<Alternate<PushPull>>,
    ),
>;

pub type PwmTimer3 = PwmHz<
    pac::TIM3,
    Tim3NoRemap,
    (Ch<C1>, Ch<C2>, Ch<C3>, Ch<C4>),
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

pub struct Configs {
    data: Vec<Config, 8>,
}

impl Default for Configs {
    fn default() -> Self {
        let mut data: Vec<Config, 8> = Vec::new();
        data.push(Config::new(FanId::F0)).ok();
        data.push(Config::new(FanId::F1)).ok();
        data.push(Config::new(FanId::F2)).ok();
        data.push(Config::new(FanId::F3)).ok();
        data.push(Config::new(FanId::F4)).ok();
        data.push(Config::new(FanId::F5)).ok();
        data.push(Config::new(FanId::F6)).ok();
        data.push(Config::new(FanId::F7)).ok();
        Self { data }
    }
}

impl AsRef<Vec<Config, 8>> for Configs {
    fn as_ref(&self) -> &Vec<Config, 8> {
        &self.data
    }
}

impl Configs {
    pub fn from_flash(flash: &mut flash::Parts) -> Self {
        let writer = get_writer(flash);
        let mut configs = Configs::default();

        let bytes = match writer.read(FLASH_START_OFFSET, SZ_1K as usize) {
            Ok(it) => it,
            _ => return configs,
        };
        for c in configs.data.iter_mut() {
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

    pub fn set(&mut self, config: Config) {
        for c in self.data.iter_mut() {
            if c.id == config.id {
                debug!("setting new config {:?}", config);
                *c = config;
                break;
            }
        }
    }

    pub fn save_to_flash(&self, flash: &mut flash::Parts) {
        let mut writer = get_writer(flash);
        let mut buff: Vec<u8, BUF_SIZE> = Vec::new();

        self.as_ref().iter().for_each(|f| {
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

    pub fn get(&self, fan_id: FanId) -> Option<&Config> {
        self.data.iter().find(|&&c| c.id == fan_id)
    }
}

fn get_writer(flash: &mut flash::Parts) -> flash::FlashWriter {
    let writer = flash.writer(SectorSize::Sz1K, FlashSize::Sz64K);
    writer
}

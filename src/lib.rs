#![no_std]

pub mod controller;
pub mod serial_handler;
pub mod tacho;
use core::{
    ops::Deref,
    sync::atomic::{AtomicUsize, Ordering},
};

use controller::{Config, FanId, FLASH_START_OFFSET};
use defmt::{debug, Format};
// global logger
use defmt_rtt as _;
use heapless::Vec;
use panic_probe as _;
use postcard::{from_bytes, to_vec};
use serde::{Deserialize, Serialize};
use stm32f1xx_hal::{
    flash::{self, FlashSize, SectorSize, SZ_1K},
    gpio::{
        gpioa::{PA0, PA1, PA2, PA3},
        Alternate, PushPull,
    },
    pac,
    timer::{Ch, PwmHz, Tim2NoRemap, C1, C2, C3, C4},
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

#[derive(Format, Deserialize, Serialize)]
pub struct Configs {
    data: Vec<Config, 4>,
    pub persistent: bool,
}

impl Default for Configs {
    fn default() -> Self {
        let mut data: Vec<_, 4> = Vec::new();
        data.push(Config::new(FanId::F1)).ok();
        data.push(Config::new(FanId::F2)).ok();
        data.push(Config::new(FanId::F3)).ok();
        data.push(Config::new(FanId::F4)).ok();

        Self {
            data,
            persistent: true,
        }
    }
}

impl AsRef<Vec<Config, 4>> for Configs {
    fn as_ref(&self) -> &Vec<Config, 4> {
        &self.data
    }
}

impl Configs {
    pub fn from_flash(flash: &mut flash::Parts) -> Self {
        let writer = get_writer(flash);

        let bytes = match writer.read(FLASH_START_OFFSET, SZ_1K as usize) {
            Ok(it) => it,
            _ => {
                defmt::error!("failed to read flash");
                return Configs::default();
            }
        };

        if let Ok(configs) = from_bytes::<Configs>(bytes) {
            if configs.is_valid() {
                return configs;
            }
        }
        Configs::default()
    }

    fn is_valid(&self) -> bool {
        self.as_ref().iter().all(|c| c.is_valid())
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
        // let mut buff: Vec<u8, BUF_SIZE> = Vec::new();

        if let Ok(buff) = to_vec::<Configs, 88>(&self) {
            defmt::debug!("{}", self);
            defmt::debug!("Saving {} to flash", buff);
            defmt::debug!("length {}", buff.len());
            writer.page_erase(FLASH_START_OFFSET).ok();

            writer.write(FLASH_START_OFFSET, buff.deref()).ok();
        }

        // self.as_ref().iter().for_each(|f| {
        //     let mut ser: Vec<u8, CONFIG_SIZE> = to_vec(f).unwrap();
        //     debug!("ser {}", ser.deref());
        //     ser.resize_default(CONFIG_SIZE).ok();
        //     debug!("after resize {}", ser.deref());

        //     buff.extend(ser.into_iter());

        //     debug!("buf {}", buff.deref());
        // });

        // writer.page_erase(FLASH_START_OFFSET).ok();

        // writer.write(FLASH_START_OFFSET, buff.deref()).ok();
    }

    pub fn get(&self, fan_id: FanId) -> Option<&Config> {
        self.data.iter().find(|&&c| c.id == fan_id)
    }
}

fn get_writer(flash: &mut flash::Parts) -> flash::FlashWriter {
    let writer = flash.writer(SectorSize::Sz1K, FlashSize::Sz64K);
    writer
}

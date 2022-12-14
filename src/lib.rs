#![no_std]
pub mod controller;
pub mod serial_handler;
pub mod tacho;

// global logger
use defmt_rtt as _;
use opilio_lib::{error::Error, Config, Result};
use panic_probe as _;
use stm32f1xx_hal::{
    flash::{self, FlashSize, SectorSize, SZ_1K},
    gpio::{
        gpioa::{PA0, PA1, PA2, PA3},
        Alternate, PushPull,
    },
    pac,
    timer::{Ch, PwmHz, Tim2NoRemap, C1, C2, C3, C4},
};

/// start address: 0x08000000
/// used by program: 60 KIB
/// we can use the rest from the 64K memory (4kb) for storage
const FLASH_START_OFFSET: u32 = 0x0C800;

// same panicking *behaviour* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is
// invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
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

pub trait FlashOps {
    fn from_flash(flash: &mut flash::Parts) -> Result<Config>;
    fn save_to_flash(&self, flash: &mut flash::Parts) -> Result<()>;
}

impl FlashOps for Config {
    fn from_flash(flash: &mut flash::Parts) -> Result<Self> {
        let writer = get_writer(flash);

        let bytes = match writer.read(FLASH_START_OFFSET, SZ_1K as usize) {
            Ok(it) => it,
            _ => {
                defmt::error!("failed to read flash");
                return Err(Error::FlashRead);
            }
        };

        defmt::info!("Bytes {} from flash", bytes);
        Self::from_bytes(bytes)
    }

    fn save_to_flash(&self, flash: &mut flash::Parts) -> Result<()> {
        let mut writer = get_writer(flash);

        let mut buff = self.to_vec()?;
        // byte align data if it ends at an odd length
        if buff.len() & 0x1 != 0 {
            buff.push(0x0FF).ok();
        }

        writer
            .page_erase(FLASH_START_OFFSET)
            .map_err(|_| Error::FlashErase)?;

        writer
            .write(FLASH_START_OFFSET, &buff)
            .map_err(|_| Error::FlashWrite)?;

        defmt::info!("Saved");
        Ok(())
    }
}

fn get_writer(flash: &mut flash::Parts) -> flash::FlashWriter {
    flash.writer(SectorSize::Sz1K, FlashSize::Sz64K)
}

use core::ops::Deref;

use super::consts;
use defmt::Format;
use heapless::Vec;
use postcard::{from_bytes, to_vec};
use serde::{Deserialize, Serialize};
use stm32f1xx_hal::flash::{FlashWriter, SZ_1K};

pub const CONFIG_SIZE: usize = 13; // core::mem::size_of::<Config>();
pub const BUF_SIZE: usize = CONFIG_SIZE * 4;

#[derive(Format, Deserialize, Serialize)]
#[repr(C)]
pub struct Config {
    id: u8,
    min_duty: u16,
    max_duty: u16,
    min_temp: f32,
    max_temp: f32,
}

impl Config {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            min_duty: consts::MIN_DUTY,
            max_duty: consts::MAX_DUTY,
            min_temp: consts::MIN_TEMP,
            max_temp: consts::MAX_TEMP,
        }
    }

    fn is_valid(&self) -> bool {
        self.min_duty >= consts::MIN_DUTY
            && self.max_duty <= consts::MAX_DUTY
            && self.min_temp >= consts::MIN_TEMP
            && self.max_temp <= consts::MAX_TEMP
    }

    fn offset(&self) -> usize {
        self.id as usize * CONFIG_SIZE
    }
}

pub struct FanControl {
    configs: Vec<Config, 4>,
    //config_size: usize,
}

impl Default for FanControl {
    fn default() -> Self {
        let mut configs: Vec<Config, 4> = Vec::new();
        configs.push(Config::new(0)).ok();
        configs.push(Config::new(1)).ok();
        configs.push(Config::new(2)).ok();
        configs.push(Config::new(3)).ok();

        Self { configs }
    }
}

impl FanControl {
    pub fn print(&self) {
        self.configs.iter().for_each(|c| {
            defmt::println!("{}", c);
        })
    }

    pub fn tem_to_duty(&self, id: usize, temp: f32) -> u16 {
        let id = id.min(4);
        let conf = &self.configs[id];
        if temp <= conf.min_temp {
            0 // stop the fan if tem is really low
        } else if temp >= conf.max_temp {
            conf.max_duty
        } else {
            ((conf.max_duty - conf.max_duty) as f32 * (temp - conf.min_temp)
                / (conf.max_temp - conf.min_temp)
                + conf.max_duty as f32) as u16
        }
    }

    pub fn load_from_flash(&mut self, writer: &mut FlashWriter) {
        if let Ok(bytes) = writer.read(consts::FLASH_START_OFFSET, SZ_1K as usize) {
            self.configs.iter_mut().for_each(|c| {
                let start = c.offset();
                let end = start + CONFIG_SIZE;
                let chunk = &bytes[start..end];
                defmt::println!("[{}, {}]: {}", start, end, chunk);
                if let Ok(config) = from_bytes::<Config>(chunk) {
                    defmt::debug!("From memory {}", config);
                    if config.is_valid() {
                        *c = config;
                    }
                }
            });
        }
    }

    pub fn save_to_flash(&mut self, writer: &mut FlashWriter) {
        let mut buff: Vec<u8, BUF_SIZE> = Vec::new();

        self.configs.iter().for_each(|f| {
            let ser: Vec<u8, CONFIG_SIZE> = to_vec(f).unwrap();
            // defmt::debug!("ser {}", ser.deref());
            buff.extend(ser.into_iter());
            // defmt::debug!("buf {}", buff.deref());
        });

        writer.change_verification(false);
        writer.page_erase(consts::FLASH_START_OFFSET).ok();

        writer.write(consts::FLASH_START_OFFSET, buff.deref()).ok();
    }
}

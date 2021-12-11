use core::ops::{Deref, Range};

use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::Format;
use embedded_hal::Pwm;
use heapless::Vec;
use micromath::F32Ext;
use postcard::{from_bytes, to_vec};
use serde::{Deserialize, Serialize};
use stm32f1xx_hal::{
    adc::Adc,
    flash::{FlashWriter, SZ_1K},
    gpio::{gpioa::PA4, Analog},
    pac::ADC1,
};

use crate::{Error, PwmTimer2, PwmTimer3, Result};

use stm32f1xx_hal::pwm::Channel;

use core::mem::size_of;

pub const MAX_DUTY_PERCENT: f32 = 100.0;
pub const MIN_DUTY_PERCENT: f32 = 10.0; // 10% usually when a pwm fan starts to spin
/// 10k resistor measured resistance in Ohms
pub const R_10K: f32 = 10000.0;
/// voltage
pub const V_SUPPLY: f32 = 3.3;
/// B-coefficient of the thermistor (guessed)
pub const B_PARAM: f32 = 3700.0; //3200.0;
/// 0*C in kelvin
pub const ZERO_K_IN_C: f32 = 273.15;

pub const MIN_TEMP: f32 = 20.0;
pub const MAX_TEMP: f32 = 40.0;
// Analog to digital resolution
pub const ADC_RESOLUTION: f32 = 4096.0;

/// start address: 0x08000000
/// used by program: 50 KIB or 51200 bytes
/// we can use the rest of the 64K memory (14kb) for storage
pub const FLASH_START_OFFSET: u32 = 0x0C800;

pub const MAX_DUTY_PWM: u16 = 2000;

pub const NO_OF_FANS: usize = 8;
pub const CONFIG_SIZE: usize = size_of::<Config>();
pub const FLASH_BUF_SIZE: usize = CONFIG_SIZE * NO_OF_FANS;
pub const PWM_CHANNELS: [Channel; 4] =
    [Channel::C1, Channel::C2, Channel::C3, Channel::C4];

static mut CONFIGS: Option<Vec<Config, NO_OF_FANS>> = None;

#[derive(Format, Deserialize, Serialize)]
pub struct Config {
    id: FanId,
    enabled: bool,
    min_duty: f32,
    max_duty: f32,
    min_temp: f32,
    max_temp: f32,
}

#[derive(Format, Copy, Clone, Deserialize, Serialize)]
pub enum FanId {
    F0 = 0,
    F1 = 1,
    F2 = 2,
    F3 = 3,
    F4 = 4,
    F5 = 5,
    F6 = 6,
    F7 = 7,
}

impl Config {
    fn new(id: FanId) -> Self {
        Self {
            id,
            enabled: true,
            min_duty: MIN_DUTY_PERCENT,
            max_duty: MAX_DUTY_PERCENT,
            min_temp: MIN_TEMP,
            max_temp: MAX_TEMP,
        }
    }

    fn is_valid(&self) -> bool {
        self.min_duty >= MIN_DUTY_PERCENT
            && self.max_duty <= MAX_DUTY_PERCENT
            && self.min_temp >= MIN_TEMP
            && self.max_temp <= MAX_TEMP
    }

    fn offset_range(&self) -> Range<usize> {
        let start = self.id as usize * CONFIG_SIZE;
        let end = start + CONFIG_SIZE;
        start..end
    }
}

fn configs() -> &'static mut [Config] {
    unsafe { CONFIGS.as_deref_mut().unwrap() }
}

fn initialize_configs() {
    let mut configs: Vec<Config, 8> = Vec::new();
    configs.push(Config::new(FanId::F0)).ok();
    configs.push(Config::new(FanId::F1)).ok();
    configs.push(Config::new(FanId::F2)).ok();
    configs.push(Config::new(FanId::F3)).ok();
    configs.push(Config::new(FanId::F4)).ok();
    configs.push(Config::new(FanId::F5)).ok();
    configs.push(Config::new(FanId::F6)).ok();
    configs.push(Config::new(FanId::F7)).ok();

    unsafe {
        CONFIGS = Some(configs);
    }
}

pub struct Controller<'flash> {
    adc: Adc<ADC1>,
    thermistor_pin: PA4<Analog>,
    timer2: PwmTimer2,
    timer3: PwmTimer3,
    max_pwm_duty: u16,
    writer: FlashWriter<'flash>,
}

impl<'flash> Controller<'flash> {
    pub fn new(
        timer2: PwmTimer2,
        timer3: PwmTimer3,
        adc: Adc<ADC1>,
        thermistor_pin: PA4<Analog>,
        writer: FlashWriter<'flash>,
    ) -> Self {
        initialize_configs();
        let max_pwm_duty = timer2.get_max_duty();

        let mut controller = Self {
            thermistor_pin,
            adc,
            max_pwm_duty,
            timer2,
            timer3,
            writer,
        };

        controller.setup_pwm();

        if controller.load_from_flash().is_err() {}

        controller
    }

    fn setup_pwm(&mut self) {
        let min_duty = self.max_pwm_duty * 10 / 100;

        for channel in PWM_CHANNELS {
            self.timer2.enable(channel);
            self.timer3.enable(channel);
            self.timer2.set_duty(channel, min_duty);
            self.timer3.set_duty(channel, min_duty);
        }
    }

    pub fn adjust_pwm(&mut self) {
        if let Some(temp) = self.get_temperature() {
            defmt::println!("Temp: {}", temp);
            for conf in configs() {
                self.set_duty(&conf, temp);
            }
        }
    }

    pub fn get_temperature(&mut self) -> Option<f32> {
        if let Ok(adc1_reading) = self.adc.read(&mut self.thermistor_pin) {
            Some(adc_reading_to_temp(adc1_reading))
        } else {
            None
        }
    }

    pub fn print(&self) {
        configs().iter().for_each(|c| {
            defmt::println!("{}", c);
        })
    }

    fn set_duty(&mut self, conf: &Config, temp: f32) {
        if conf.enabled {
            defmt::debug!("Setting duty for: {:#?}", conf);
            let duty_percent = if temp <= conf.min_temp {
                0.0 // stop the fan if tem is really low
            } else if temp >= conf.max_temp {
                conf.max_duty
            } else {
                (conf.max_duty - conf.min_duty) * (temp - conf.min_temp)
                    / (conf.max_temp - conf.min_temp)
                    + conf.min_duty
            };

            let duty_to_set = self.max_pwm_duty / 100 * duty_percent as u16;

            self.set_channel_duty(conf.id, duty_to_set)
        }
    }

    fn set_channel_duty(&mut self, id: FanId, duty: u16) {
        let id = id as usize;
        match id {
            0..=3 => self.timer2.set_duty(PWM_CHANNELS[id], duty),
            4..=7 => self.timer2.set_duty(PWM_CHANNELS[id - 4], duty),
            _ => unreachable!("shouldn't reach "),
        }
    }

    pub fn load_from_flash(&mut self) -> Result<()> {
        if let Ok(bytes) = self.writer.read(FLASH_START_OFFSET, SZ_1K as usize)
        {
            for c in configs().iter_mut() {
                let range = c.offset_range();
                let chunk = &bytes[c.offset_range()];
                defmt::println!("range {}; chunk: {}", range, chunk);
                let config = from_bytes::<Config>(chunk)
                    .map_err(|_| Error::Deserialize)?;

                defmt::debug!("From memory {}", config);
                if config.is_valid() {
                    *c = config;
                }
            }
        }
        Ok(())
    }

    pub fn save_to_flash(&mut self) {
        let mut buff: Vec<u8, FLASH_BUF_SIZE> = Vec::new();

        configs().iter().for_each(|f| {
            let mut ser: Vec<u8, CONFIG_SIZE> = to_vec(f).unwrap();
            defmt::debug!("ser {}", ser.deref());
            ser.resize_default(CONFIG_SIZE).unwrap();
            defmt::debug!("after resize {}", ser.deref());

            buff.extend(ser.into_iter());

            defmt::debug!("buf {}", buff.deref());
        });

        self.writer.change_verification(false);
        self.writer.page_erase(FLASH_START_OFFSET).ok();

        self.writer.write(FLASH_START_OFFSET, buff.deref()).ok();
    }
}

fn adc_reading_to_temp(adc_reading: u16) -> f32 {
    let v_out: f32 = adc_reading as f32 * V_SUPPLY / ADC_RESOLUTION;
    defmt::trace!("v_out {}", v_out);
    defmt::trace!("adc_reading: {}", adc_reading);

    let r_ntc = (v_out * R_10K) / (V_SUPPLY - v_out);
    defmt::trace!("rt {}", r_ntc);

    let t_25_c_in_k = ZERO_K_IN_C + 25.0;

    let temp_k = (t_25_c_in_k * B_PARAM)
        / (t_25_c_in_k * (r_ntc / R_10K).ln() + B_PARAM);

    defmt::trace!("k {}", temp_k);
    temp_k - ZERO_K_IN_C
}

use core::ops::Deref;

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
    pwm::Channel,
};

use crate::{PwmTimer2, PwmTimer3};

use super::consts;

// current size of Config is 20 bytes but we have extra space for future
// expansion
pub const CHUNK_SIZE: usize = 32;
pub const BUF_SIZE: usize = CHUNK_SIZE * 4;
pub const PWM_CHANNELS: [Channel; 4] =
    [Channel::C1, Channel::C2, Channel::C3, Channel::C4];

static mut CONFIGS: Option<Vec<Config, 8>> = None;

#[derive(Format, Deserialize, Serialize)]
pub struct Config {
    id: FanId,
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
            min_duty: consts::MIN_DUTY_PERCENT,
            max_duty: consts::MAX_DUTY_PERCENT,
            min_temp: consts::MIN_TEMP,
            max_temp: consts::MAX_TEMP,
        }
    }

    fn is_valid(&self) -> bool {
        self.min_duty >= consts::MIN_DUTY_PERCENT
            && self.max_duty <= consts::MAX_DUTY_PERCENT
            && self.min_temp >= consts::MIN_TEMP
            && self.max_temp <= consts::MAX_TEMP
    }

    fn offset(&self) -> usize {
        self.id as usize * CHUNK_SIZE
    }
}

fn configs() -> &'static mut [Config] {
    unsafe { CONFIGS.as_deref_mut().unwrap() }
}

pub struct Controller {
    adc: Adc<ADC1>,
    thermistor_pin: PA4<Analog>,
    timer2: PwmTimer2,
    timer3: PwmTimer3,
    max_pwm_duty: u16,
}

impl Controller {
    pub fn new(
        timer2: PwmTimer2,
        timer3: PwmTimer3,
        adc: Adc<ADC1>,
        thermistor_pin: PA4<Analog>,
    ) -> Self {
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

        let max_pwm_duty = timer2.get_max_duty();

        let mut controller = Self {
            thermistor_pin,
            adc,
            max_pwm_duty,
            timer2,
            timer3,
        };

        controller.setup_pwm();
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
            Some(voltage_to_temp(adc1_reading))
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

    fn set_channel_duty(&mut self, id: FanId, duty: u16) {
        let id = id as usize;
        match id {
            0..=3 => self.timer2.set_duty(PWM_CHANNELS[id], duty),
            4..=7 => self.timer2.set_duty(PWM_CHANNELS[id - 4], duty),
            _ => unreachable!("shouldn't reach "),
        }
    }

    pub fn load_from_flash(&mut self, writer: &mut FlashWriter) {
        if let Ok(bytes) =
            writer.read(consts::FLASH_START_OFFSET, SZ_1K as usize)
        {
            configs().iter_mut().for_each(|c| {
                let start = c.offset();
                let end = start + CHUNK_SIZE;
                let chunk = &bytes[start..end];
                defmt::println!("range [{}, {}]; chunk: {}", start, end, chunk);
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

        configs().iter().for_each(|f| {
            let mut ser: Vec<u8, CHUNK_SIZE> = to_vec(f).unwrap();
            defmt::debug!("ser {}", ser.deref());
            ser.resize_default(CHUNK_SIZE).unwrap();
            defmt::debug!("ser {}", ser.deref());

            buff.extend(ser.into_iter());

            defmt::debug!("buf {}", buff.deref());
        });

        writer.change_verification(false);
        // writer.page_erase(consts::FLASH_START_OFFSET).ok();

        // writer.write(consts::FLASH_START_OFFSET, buff.deref()).ok();
    }
}

fn voltage_to_temp(adc_reading: u16) -> f32 {
    let v_out: f32 =
        adc_reading as f32 * consts::V_SUPPLY / consts::ADC_RESOLUTION;
    defmt::trace!("v_out {}", v_out);
    defmt::trace!("adc_reading: {}", adc_reading);

    let r_ntc = (v_out * consts::R_10K) / (consts::V_SUPPLY - v_out);
    defmt::trace!("rt {}", r_ntc);

    let t_25_c_in_k = consts::ZERO_K_IN_C + 25.0;

    let temp_k = (t_25_c_in_k * consts::B_PARAM)
        / (t_25_c_in_k * (r_ntc / consts::R_10K).ln() + consts::B_PARAM);

    defmt::trace!("k {}", temp_k);
    temp_k - consts::ZERO_K_IN_C
}

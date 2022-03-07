use core::{mem::size_of, ops::Range};

use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::{trace, Format};
use embedded_hal::Pwm;
use heapless::Vec;
use micromath::F32Ext;
// use postcard::{from_bytes, to_vec};
use serde::{Deserialize, Serialize};
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{gpioa::PA4, Analog},
    pac::ADC1,
    pwm::Channel,
};

use crate::{PwmTimer2, PwmTimer3};

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
// pub const FLASH_START_OFFSET: u32 = 0x0C800;
pub const FLASH_START_OFFSET: u32 = 0xF000;
pub const MAX_DUTY_PWM: u16 = 2000;
pub const NO_OF_FANS: usize = 8;
pub const CONFIG_SIZE: usize = size_of::<Config>();
pub const BUF_SIZE: usize = CONFIG_SIZE * NO_OF_FANS;

#[derive(Copy, Clone, Format, Deserialize, Serialize)]
pub struct Config {
    id: FanId,
    enabled: bool,
    min_duty: f32,
    max_duty: f32,
    min_temp: f32,
    max_temp: f32,
}

#[derive(Format, Copy, Debug, Clone, Deserialize, Serialize, PartialEq)]
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

impl FanId {
    pub fn get_channel(&self) -> Channel {
        use Channel::*;
        use FanId::*;

        match self {
            F0 => C4,
            F1 => C3,
            F2 => C2,
            F3 => C1,
            F4 => C4,
            F5 => C3,
            F6 => C2,
            F7 => C1,
        }
    }

    pub fn next_id(&self) -> Self {
        use FanId::*;
        match self {
            F7 => F0,
            F0 => F1,
            F1 => F2,
            F2 => F3,
            F3 => F4,
            F4 => F5,
            F5 => F6,
            F6 => F7,
        }
    }
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

    pub fn is_valid(&self) -> bool {
        self.min_duty >= MIN_DUTY_PERCENT
            && self.max_duty <= MAX_DUTY_PERCENT
            && self.min_temp >= MIN_TEMP
            && self.max_temp <= MAX_TEMP
    }

    pub fn offset_range(&self) -> Range<usize> {
        let start = self.id as usize * CONFIG_SIZE;
        let end = start + CONFIG_SIZE;
        start..end
    }
}

pub fn default_configs() -> Vec<Config, 8> {
    let mut configs: Vec<Config, 8> = Vec::new();
    configs.push(Config::new(FanId::F0)).ok();
    configs.push(Config::new(FanId::F1)).ok();
    configs.push(Config::new(FanId::F2)).ok();
    configs.push(Config::new(FanId::F3)).ok();
    configs.push(Config::new(FanId::F4)).ok();
    configs.push(Config::new(FanId::F5)).ok();
    configs.push(Config::new(FanId::F6)).ok();
    configs.push(Config::new(FanId::F7)).ok();

    configs
}

pub fn default_rpm() -> Vec<u32, 8> {
    let mut rpms: Vec<u32, 8> = Vec::new();
    rpms.push(0).ok();
    rpms.push(0).ok();
    rpms.push(0).ok();
    rpms.push(0).ok();
    rpms.push(0).ok();
    rpms.push(0).ok();
    rpms.push(0).ok();
    rpms.push(0).ok();
    rpms
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
        use Channel::*;
        let min_duty = self.max_pwm_duty * 10 / 100;

        for channel in [C1, C2, C3, C4] {
            self.timer2.enable(channel);
            self.timer3.enable(channel);
            self.timer2.set_duty(channel, min_duty);
            self.timer3.set_duty(channel, min_duty);
        }
    }

    pub fn adjust_pwm(&mut self, configs: &Vec<Config, 8>) {
        if let Some(temp) = self.get_temperature() {
            defmt::info!("Temp: {}", temp);
            for conf in configs {
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

    // pub fn print(&self) {
    //     configs().iter().for_each(|c| {
    //         debug!("{}", c);
    //     })
    // }

    fn set_duty(&mut self, conf: &Config, temp: f32) {
        if conf.enabled {
            // debug!("Setting duty for: {:#?}", conf);
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

            trace!("duty {}", duty_to_set);
            self.set_channel_duty(conf.id, duty_to_set)
        }
    }

    fn set_channel_duty(&mut self, id: FanId, duty: u16) {
        if id as usize <= 3 {
            self.timer3.set_duty(id.get_channel(), duty);
        } else {
            self.timer2.set_duty(id.get_channel(), duty)
        }
    }
}

fn adc_reading_to_temp(adc_reading: u16) -> f32 {
    let v_out: f32 = adc_reading as f32 * V_SUPPLY / ADC_RESOLUTION;
    trace!("v_out {}", v_out);
    trace!("adc_reading: {}", adc_reading);

    let r_ntc = (v_out * R_10K) / (V_SUPPLY - v_out);
    trace!("rt {}", r_ntc);

    let t_25_c_in_k = ZERO_K_IN_C + 25.0;

    let temp_k = (t_25_c_in_k * B_PARAM)
        / (t_25_c_in_k * (r_ntc / R_10K).ln() + B_PARAM);

    trace!("k {}", temp_k);
    temp_k - ZERO_K_IN_C
}

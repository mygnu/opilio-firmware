use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::{debug, trace};
use micromath::F32Ext;
use shared::{Config, Configs, FanId};
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{gpioa::PA4, Analog, Output, PushPull, PB12, PB13, PB14, PB15},
    pac::ADC1,
    timer::Channel,
};

use crate::PwmTimer2;

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

pub const MIN_TEMP: f32 = 15.0;
pub const MAX_TEMP: f32 = 40.0;
// Analog to digital resolution
pub const ADC_RESOLUTION: f32 = 4096.0;

/// start address: 0x08000000
/// used by program: 60 KIB
/// we can use the rest from the 64K memory (4kb) for storage
pub const FLASH_START_OFFSET: u32 = 0xF000;
pub const MAX_DUTY_PWM: u16 = 2000;

trait ChannelMap {
    fn channel(&self) -> Channel;
}

impl ChannelMap for FanId {
    fn channel(&self) -> Channel {
        use Channel::*;
        use FanId::*;

        match self {
            F1 => C4,
            F2 => C3,
            F3 => C2,
            F4 => C1,
        }
    }
}

pub struct Controller {
    adc: Adc<ADC1>,
    max_pwm_duty: u16,
    fan_enable: PB12<Output<PushPull>>,
    pump_enable: PB13<Output<PushPull>>,
    red_led: PB14<Output<PushPull>>,
    blue_led: PB15<Output<PushPull>>,
    pwm_timer: PwmTimer2,
    thermistor_pin: PA4<Analog>,
}

impl Controller {
    pub fn new(
        timer2: PwmTimer2,
        adc: Adc<ADC1>,
        thermistor_pin: PA4<Analog>,
        fan_enable: PB12<Output<PushPull>>,
        pump_enable: PB13<Output<PushPull>>,
        red_led: PB14<Output<PushPull>>,
        blue_led: PB15<Output<PushPull>>,
    ) -> Self {
        let max_pwm_duty = timer2.get_max_duty();

        let mut controller = Self {
            thermistor_pin,
            adc,
            max_pwm_duty,
            pwm_timer: timer2,
            fan_enable,
            pump_enable,
            red_led,
            blue_led,
        };
        controller.setup_pwm();

        controller
    }

    pub fn standby_mode(&mut self) {
        if self.fan_enable.is_set_high() {
            debug!("turning off fan and pump");
            self.fan_enable.set_low();
            self.pump_enable.set_low();
        }

        if self.blue_led.is_set_low() {
            self.blue_led.set_high();
        }
    }
    pub fn active_mode(&mut self) {
        if self.fan_enable.is_set_low() {
            debug!("enabling fan and pump");
            self.fan_enable.set_high();
            self.pump_enable.set_high();
        }

        if self.blue_led.is_set_high() {
            self.blue_led.set_low();
        }
    }

    fn setup_pwm(&mut self) {
        let min_duty = self.max_pwm_duty * 10 / 100;
        use Channel::*;
        for channel in [C1, C2, C3, C4] {
            self.pwm_timer.enable(channel);
            self.pwm_timer.set_duty(channel, min_duty);
        }
    }

    pub fn adjust_pwm(&mut self, configs: &Configs) {
        let temp = self.get_temp();
        defmt::info!("Temp: {}", temp);
        for conf in configs.as_ref() {
            self.set_duty(&conf, temp);
        }
    }

    pub fn get_temp(&mut self) -> f32 {
        if let Ok(adc1_reading) = self.adc.read(&mut self.thermistor_pin) {
            self.red_led.set_low();
            adc_reading_to_temp(adc1_reading)
        } else {
            self.red_led.set_high();
            30.0 // assume we are running hot
        }
    }

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
        self.pwm_timer.set_duty(id.channel(), duty)
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

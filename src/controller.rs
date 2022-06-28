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

/// 10k resistor measured resistance in Ohms
const R_10K: f32 = 10000.0;
/// voltage to the VREF
const V_SUPPLY: f32 = 3.3;
/// B-coefficient of the thermistor (guessed)
const B_PARAM: f32 = 3700.0;

/// 0*C in kelvin
const ZERO_K_IN_C: f32 = 273.15;
// Analog to digital resolution
const ADC_RESOLUTION: f32 = 4096.0;

trait ChannelMap {
    fn channel(&self) -> Channel;
}

impl ChannelMap for FanId {
    fn channel(&self) -> Channel {
        use Channel::*;
        use FanId::*;

        match self {
            F1 => C1,
            F2 => C2,
            F3 => C3,
            F4 => C4,
        }
    }
}

/// controller for most of the output of opilio
/// Responsible for
/// - Reading thermistor and converting it to temperature in C
/// - Enable/Disable fan and pump mosfets (via mosfet drivers)
/// - Controls Red an Blue single leds
/// - Controls PWM signal for Fans and pumps
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
        mut timer2: PwmTimer2,
        adc: Adc<ADC1>,
        thermistor_pin: PA4<Analog>,
        fan_enable: PB12<Output<PushPull>>,
        pump_enable: PB13<Output<PushPull>>,
        red_led: PB14<Output<PushPull>>,
        blue_led: PB15<Output<PushPull>>,
    ) -> Self {
        let max_pwm_duty = timer2.get_max_duty();
        let min_duty = max_pwm_duty * 10 / 100;

        // enable all channel as pwm output with minimum 10% duty cycle
        for channel in [Channel::C1, Channel::C2, Channel::C3, Channel::C4] {
            timer2.enable(channel);
            timer2.set_duty(channel, min_duty);
        }

        Self {
            thermistor_pin,
            adc,
            max_pwm_duty,
            pwm_timer: timer2,
            fan_enable,
            pump_enable,
            red_led,
            blue_led,
        }
    }

    /// puts fan and pump mosfets in off mode
    /// blue signal led is set high
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

    /// puts fan and pump mosfets in on mode
    /// blue signal led is set low
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

    /// Adjust PWM on all channels according to the configuration
    /// and temperature reading
    pub fn adjust_pwm(&mut self, configs: &Configs) {
        let temp = self.get_temp();
        defmt::info!("Temp: {}", temp);
        for conf in configs.as_ref() {
            self.set_duty(&conf, temp);
        }
    }

    /// reads temperature in celsius degrees
    /// if there is an error we assume that its 30 degrees
    /// red led is turned on in case of error
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
            self.pwm_timer.set_duty(conf.id.channel(), duty_to_set)
        }
    }
}

/// converts ADC reading value to degrees celsius
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

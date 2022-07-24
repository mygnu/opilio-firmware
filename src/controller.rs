use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::{debug, trace};
use micromath::F32Ext;
use opilio_lib::{error::Error, Config, Id, Result};
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{gpioa::PA4, Analog, Output, PushPull, PA5, PB12, PB13, PB14, PB15},
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

impl ChannelMap for Id {
    fn channel(&self) -> Channel {
        use Channel::*;
        use Id::*;

        match self {
            P1 => C1,
            F1 => C2,
            F2 => C3,
            F3 => C4,
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
    max_duty_value: u16,
    fan_enable: PB12<Output<PushPull>>,
    pump_enable: PB13<Output<PushPull>>,
    red_led: PB14<Output<PushPull>>,
    blue_led: PB15<Output<PushPull>>,
    pwm_timer: PwmTimer2,
    liquid_thermistor: PA4<Analog>,
    ambient_thermistor: PA5<Analog>,
    liquid_temp: f32,
    ambient_temp: f32,
}

impl Controller {
    pub fn new(
        mut timer2: PwmTimer2,
        adc: Adc<ADC1>,
        liquid_thermistor: PA4<Analog>,
        ambient_thermistor: PA5<Analog>,
        fan_enable: PB12<Output<PushPull>>,
        pump_enable: PB13<Output<PushPull>>,
        red_led: PB14<Output<PushPull>>,
        blue_led: PB15<Output<PushPull>>,
    ) -> Self {
        let max_duty_value = timer2.get_max_duty();
        let min_duty = max_duty_value * 10 / 100;

        // enable all channel as pwm output with minimum 10% duty cycle
        for channel in [Channel::C1, Channel::C2, Channel::C3, Channel::C4] {
            timer2.enable(channel);
            timer2.set_duty(channel, min_duty);
        }

        Self {
            liquid_thermistor,
            ambient_thermistor,
            adc,
            max_duty_value,
            pwm_timer: timer2,
            fan_enable,
            pump_enable,
            red_led,
            blue_led,
            liquid_temp: 22.0,
            ambient_temp: 22.0,
        }
    }

    /// puts fan and pump mosfets in off mode
    /// blue signal led is set high
    pub fn standby_mode(&mut self) {
        // shut down pwm
        for channel in [Channel::C1, Channel::C2, Channel::C3, Channel::C4] {
            self.pwm_timer.set_duty(channel, 0);
        }
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
    pub fn adjust_pwm(&mut self, config: &Config) {
        let (liquid_temp, ambient_temp) = if self.fetch_current_temps().is_ok()
        {
            (self.get_liquid_temp(), self.get_ambient_temp())
        } else {
            // assume we are running hot
            // in case thermistor is faulty or unplugged
            (35.0, 20.0)
        };
        for setting in &config.settings {
            // turn off fans if ambient_temp is hotter, except for the pump, it
            // should never be turned off
            let duty_to_set =
                if liquid_temp > ambient_temp || setting.id == Id::P1 {
                    setting.get_duty(liquid_temp, self.max_duty_value)
                } else {
                    0
                };
            trace!("duty {}", duty_to_set);
            self.pwm_timer.set_duty(setting.id.channel(), duty_to_set);
        }
    }

    pub fn get_liquid_temp(&mut self) -> f32 {
        self.liquid_temp
    }

    pub fn get_ambient_temp(&mut self) -> f32 {
        self.ambient_temp
    }

    /// reads temperature in celsius degrees
    /// red led is turned on in case of error
    pub fn fetch_current_temps(&mut self) -> Result<()> {
        if let Ok(adc0_reading) = self.adc.read(&mut self.ambient_thermistor) {
            let new_temp = adc_reading_to_temp(adc0_reading);
            self.ambient_temp = (self.ambient_temp + new_temp) / 2.0;
        }

        if let Ok(adc1_reading) = self.adc.read(&mut self.liquid_thermistor) {
            let new_temp = adc_reading_to_temp(adc1_reading);
            // if thermistor is disconnected, temp reading is in negative
            // and we want to indicate that
            if new_temp < -20.0 {
                self.red_led.set_high();
                return Err(Error::TempRead);
            }
            self.liquid_temp = (self.liquid_temp + new_temp) / 2.0;

            self.red_led.set_low();
            Ok(())
        } else {
            self.red_led.set_high();
            Err(Error::TempRead)
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

use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::{debug, info, trace};
use micromath::F32Ext;
use opilio_lib::{
    error::Error, get_smart_duty, Config, Id, Result, SwitchMode,
    TempratureInput,
};
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{
        gpioa::PA4, Analog, Output, PushPull, PA5, PA6, PA8, PB11, PB12, PB13,
        PB14, PB15,
    },
    pac::ADC1,
    timer::Channel,
};

use crate::PwmTimer2;

/// 10k resistor measured resistance in Ohms
const R_10K: f32 = 10000.0;
/// voltage to the VREF
const V_SUPPLY: f32 = 3.2925;
/// B-coefficient of the thermistor (guessed)
const B_PARAM: f32 = 3700.0;

/// 0*C in kelvin
const ZERO_K_IN_C: f32 = 273.15;

// voltage supply / ADC resolution
const ADC_RATIO: f32 = V_SUPPLY / 4096.0;

// milliseconds
pub const TICK_PERIOD: u64 = 450_u64;

pub const DEFAULT_TEMP: f32 = 22.0;

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

pub enum Led {
    RED,
    GREEN,
    BLUE,
    ALL,
}

pub struct RgbLed {
    pub use_led: bool,
    red_led: PB14<Output<PushPull>>,
    green_led: PA8<Output<PushPull>>,
    blue_led: PB15<Output<PushPull>>,
}

impl RgbLed {
    #[inline]
    pub fn new(
        red_led: PB14<Output<PushPull>>,
        green_led: PA8<Output<PushPull>>,
        blue_led: PB15<Output<PushPull>>,
    ) -> Self {
        Self {
            use_led: true,
            red_led,
            green_led,
            blue_led,
        }
    }

    #[inline]
    pub fn exclusive_on(&mut self, led: Led) {
        if self.use_led {
            match led {
                Led::RED => {
                    self.red_led.set_low();
                    self.green_led.set_high();
                    self.blue_led.set_high();
                }
                Led::GREEN => {
                    self.red_led.set_high();
                    self.green_led.set_low();
                    self.blue_led.set_high();
                }
                Led::BLUE => {
                    self.red_led.set_high();
                    self.green_led.set_high();
                    self.blue_led.set_low();
                }
                Led::ALL => {
                    self.red_led.set_low();
                    self.green_led.set_low();
                    self.blue_led.set_low();
                }
            }
        } else {
            self.off(led)
        }
    }

    pub fn toggle(&mut self, led: Led) {
        if self.use_led {
            match led {
                Led::RED => {
                    self.red_led.toggle();
                }
                Led::GREEN => {
                    self.green_led.toggle();
                }
                Led::BLUE => {
                    self.blue_led.toggle();
                }
                Led::ALL => {
                    self.red_led.toggle();
                    self.green_led.toggle();
                    self.blue_led.toggle();
                }
            }
        } else {
            self.off(led)
        }
    }

    #[inline]
    pub fn off(&mut self, led: Led) {
        match led {
            Led::RED => {
                self.red_led.set_high();
            }
            Led::GREEN => {
                self.green_led.set_high();
            }
            Led::BLUE => {
                self.blue_led.set_high();
            }
            Led::ALL => {
                self.red_led.set_high();
                self.green_led.set_high();
                self.blue_led.set_high();
            }
        }
    }
}

pub struct Temps {
    adc: Adc<ADC1>,
    ambient_temp: f32,
    ambient_thermistor: PA4<Analog>,
    coolant_temp: f32,
    coolant_thermistor: PA5<Analog>,
    coolant_out_temp: f32,
    coolant_out_thermistor: PA6<Analog>,
}

pub enum Thermistor {
    Coolant,
    CoolantOut,
    Ambient,
}

impl Temps {
    pub fn new(
        adc: Adc<ADC1>,
        ambient_thermistor: PA4<Analog>,
        coolant_thermistor: PA5<Analog>,
        coolant_out_thermistor: PA6<Analog>,
    ) -> Self {
        Self {
            adc,
            ambient_temp: DEFAULT_TEMP,
            ambient_thermistor,
            coolant_temp: DEFAULT_TEMP,
            coolant_thermistor,
            coolant_out_temp: DEFAULT_TEMP,
            coolant_out_thermistor,
        }
    }

    pub fn read(&mut self, thermistor: Thermistor) -> Result<()> {
        match thermistor {
            Thermistor::Coolant => {
                let adc_reading = self
                    .adc
                    .read(&mut self.coolant_thermistor)
                    .map_err(|_| Error::TempRead)?;

                let new_temp = adc_reading_to_temp(adc_reading);
                self.coolant_temp = (self.coolant_temp + new_temp) / 2.0;
                if new_temp < -20.0 {
                    return Err(Error::TempRead);
                }
            }
            Thermistor::CoolantOut => {
                let adc_reading = self
                    .adc
                    .read(&mut self.coolant_out_thermistor)
                    .map_err(|_| Error::TempRead)?;

                let new_temp = adc_reading_to_temp(adc_reading);
                self.coolant_out_temp =
                    (self.coolant_out_temp + new_temp) / 2.0;
            }
            Thermistor::Ambient => {
                let adc_reading = self
                    .adc
                    .read(&mut self.ambient_thermistor)
                    .map_err(|_| Error::TempRead)?;

                let new_temp = adc_reading_to_temp(adc_reading);
                self.ambient_temp = (self.ambient_temp + new_temp) / 2.0;
            }
        }
        Ok(())
    }
}

/// controller for most of the output of opilio
/// Responsible for
/// - Reading thermistor and converting it to temperature in C
/// - Enable/Disable fan and pump MOSFETs (via MOSFET drivers)
/// - Controls Red an Blue single leds
/// - Controls PWM signal for Fans and pumps
pub struct Controller {
    temps: Temps,
    buzzer_pin: PB11<Output<PushPull>>,
    fan_power: PB13<Output<PushPull>>,
    led: RgbLed,
    max_duty_value: u16,
    pump_power: PB12<Output<PushPull>>,
    pwm_timer: PwmTimer2,
    use_buzzer: bool,
}

impl Controller {
    pub fn new(
        mut timer2: PwmTimer2,
        pump_power: PB12<Output<PushPull>>,
        fan_power: PB13<Output<PushPull>>,
        buzzer: PB11<Output<PushPull>>,
        temps: Temps,
        led: RgbLed,
    ) -> Self {
        let max_duty_value = timer2.get_max_duty();
        let min_duty = max_duty_value * 10 / 100;

        // enable all channel as pwm output with minimum 10% duty cycle
        for channel in [Channel::C1, Channel::C2, Channel::C3, Channel::C4] {
            timer2.enable(channel);
            timer2.set_duty(channel, min_duty);
        }

        Self {
            temps,
            max_duty_value,
            pwm_timer: timer2,
            pump_power,
            fan_power,
            buzzer_pin: buzzer,
            led,
            use_buzzer: false,
        }
    }

    fn buzzer_on(&mut self) {
        if self.use_buzzer {
            self.buzzer_pin.set_high();
        }
    }

    fn buzzer_off(&mut self) {
        self.buzzer_pin.set_low();
    }

    /// puts fan and pump MOSFETs in off mode
    /// blue signal led is set high
    fn standby_mode(&mut self) {
        // // shut down pwm
        for channel in [Channel::C1, Channel::C2, Channel::C3, Channel::C4] {
            self.pwm_timer.set_duty(channel, 0);
        }

        if self.pump_power.is_set_high() {
            info!("turning off pump");
            self.pump_power.set_low();
        }

        if self.fan_power.is_set_high() {
            info!("turning off fans");
            self.fan_power.set_low();
        }

        self.led.toggle(Led::BLUE);
        self.led.off(Led::GREEN);
    }

    /// puts fan and pump MOSFETs in on mode
    fn active_mode(&mut self) {
        if self.pump_power.is_set_low() {
            info!("enabling pump");
            self.pump_power.set_high();
            self.led.off(Led::BLUE);
            self.led.exclusive_on(Led::GREEN);
        }
    }

    /// Adjust PWM on all channels according to the configuration
    /// and temperature reading
    pub fn adjust_pwm(&mut self, config: &Config, standby: bool) {
        if standby {
            self.standby_mode();
            return;
        }
        self.use_buzzer = config.general.buzzer == SwitchMode::On;
        self.led.use_led = config.general.led == SwitchMode::On;

        self.active_mode();

        let (coolant_temp, ambient_temp, coolant_out_temp) =
            if self.fetch_current_temps(config.general.temp_input).is_ok() {
                self.led.off(Led::RED);
                self.buzzer_off();
                (
                    self.get_coolant_temp(),
                    self.get_ambient_temp(),
                    self.get_coolant_out_temp(),
                )
            } else {
                // assume we are running hot
                // in case thermistor is faulty or unplugged
                self.led.toggle(Led::RED);
                self.buzzer_on();

                (35.0, 20.0, 35.0)
            };
        debug!(
            "coolant {}C, coolant_out_temp {}C, ambient {}C,",
            coolant_temp, coolant_out_temp, ambient_temp
        );

        let coolant_temp = match config.general.temp_input {
            TempratureInput::Coolant => coolant_temp,
            TempratureInput::CoolantOut => coolant_out_temp,
        };

        if let Some(ref smart_mode) = config.smart_mode {
            let duty_to_set = get_smart_duty(
                coolant_temp,
                ambient_temp,
                smart_mode.trigger_above_ambient,
                smart_mode.upper_temp,
                self.max_duty_value,
                self.fan_power.is_set_high(),
            );

            if coolant_temp > smart_mode.upper_temp {
                self.led.toggle(Led::BLUE);
                self.buzzer_on();
            } else {
                self.led.off(Led::BLUE);
                self.buzzer_off();
            }

            debug!("smart duty {}", duty_to_set);

            if duty_to_set > 0 {
                self.fan_power.set_high();
                // set duty except the pump
                for channel in [Channel::C2, Channel::C3, Channel::C4] {
                    self.pwm_timer.set_duty(channel, duty_to_set);
                }
            } else {
                self.fan_power.set_low()
            }

            debug!("{}:{}", self.max_duty_value, smart_mode.pump_duty);

            let pump_duty =
                self.max_duty_value as f32 * (smart_mode.pump_duty / 100.0);
            debug!("pump duty {}", pump_duty);
            self.pwm_timer.set_duty(Channel::C1, pump_duty as u16);
        } else {
            if self.fan_power.is_set_low() {
                debug!("enabling fan");
                self.fan_power.set_high();
            }
            for setting in &config.settings {
                let duty_to_set =
                    setting.get_duty(coolant_temp, self.max_duty_value);
                debug!("{}, duty {}", setting, duty_to_set);

                self.pwm_timer.set_duty(setting.id.channel(), duty_to_set);
            }
        }
    }

    #[inline]
    pub fn get_coolant_temp(&mut self) -> f32 {
        self.temps.coolant_temp
    }

    #[inline]
    pub fn get_coolant_out_temp(&mut self) -> f32 {
        self.temps.coolant_out_temp
    }

    #[inline]
    pub fn get_ambient_temp(&mut self) -> f32 {
        self.temps.ambient_temp
    }

    /// reads temperature in celsius degrees
    fn fetch_current_temps(
        &mut self,
        selected_input: TempratureInput,
    ) -> Result<()> {
        self.temps.read(Thermistor::Ambient).ok();
        match selected_input {
            TempratureInput::Coolant => {
                self.temps.read(Thermistor::CoolantOut).ok();
                self.temps.read(Thermistor::Coolant)
            }
            TempratureInput::CoolantOut => {
                self.temps.read(Thermistor::Coolant).ok();
                self.temps.read(Thermistor::CoolantOut)
            }
        }
    }
}

/// converts ADC reading value to degrees celsius
fn adc_reading_to_temp(adc_reading: u16) -> f32 {
    let v_out: f32 = adc_reading as f32 * ADC_RATIO;
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

//! Testing PWM output for pre-defined pin combination: all pins for default mapping

// #![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use micromath::F32Ext;
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{
        gpioa::{PA0, PA1, PA2, PA3},
        Alternate, PushPull,
    },
    pac,
    pwm::{self, Channel, Pwm},
    pwm_input::{Configuration, ReadMode},
    time::U32Ext,
    timer::{Tim2NoRemap, Timer},
};
use stm32f1xx_hal::{
    flash::{FlashSize, SectorSize},
    prelude::*,
};
use water_crab::{
    consts,
    models::{FanControl, CONFIG_SIZE},
};

type PwmTimer = Pwm<
    stm32f1xx_hal::pac::TIM2,
    Tim2NoRemap,
    (pwm::C1, pwm::C2, pwm::C3, pwm::C4),
    (
        PA0<Alternate<PushPull>>,
        PA1<Alternate<PushPull>>,
        PA2<Alternate<PushPull>>,
        PA3<Alternate<PushPull>>,
    ),
>;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut dbg = p.DBGMCU;
    let mut writer = flash.writer(SectorSize::Sz1K, FlashSize::Sz64K);

    defmt::println!("CONFIG_SIZE {}", CONFIG_SIZE);

    let mut fan_control = FanControl::default();
    fan_control.print();

    fan_control.load_from_flash(&mut writer);

    fan_control.print();
    // let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);

    let clocks = rcc.cfgr.adcclk(1.mhz()).freeze(&mut flash.acr);
    defmt::println!("adc freq: {}", clocks.adcclk().0);

    // Setup ADC
    let mut adc1 = Adc::adc1(p.ADC1, &mut rcc.apb2, clocks);

    let mut timer_sys = Timer::syst(cp.SYST, &clocks).start_count_down(700.ms());

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // Configure pb0 as an analog input
    let mut ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);
    // TIM2
    let a0 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let a1 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let a2 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let a3 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);

    let pwm_output_pins = (a0, a1, a2, a3);

    let mut pwm_out: PwmTimer =
        Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1).pwm(pwm_output_pins, &mut afio.mapr, 24.khz());
    setup_pwm(&mut pwm_out);

    let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
    let pb5 = gpiob.pb5;

    let pwm_input_pins = (pb4, pb5);

    let pwm_input1 = Timer::tim3(p.TIM3, &clocks, &mut rcc.apb1).pwm_input(
        pwm_input_pins,
        &mut afio.mapr,
        &mut dbg,
        Configuration::Frequency(20.hz()),
    );

    loop {
        nb::block!(timer_sys.wait()).unwrap();
        if let Ok(adc1_reading) = adc1.read(&mut ch0) {
            let temp = get_temperature(adc1_reading);
            let duty = tem_to_duty(temp);
            defmt::info!("DUTY {}", duty);
            pwm_out.set_duty(Channel::C1, duty);
        }

        if let Ok(freq) = pwm_input1.read_frequency(ReadMode::Instant, &clocks) {
            defmt::trace!("1Htz: {}", freq.0);
            defmt::debug!("1RPM: {}", freq.0 * 30);
        }
    }
}

fn setup_pwm(pwm_out: &mut PwmTimer) {
    // Enable clock on each of the channels
    pwm_out.enable(Channel::C1);
    pwm_out.enable(Channel::C2);
    pwm_out.enable(Channel::C3);
    pwm_out.enable(Channel::C4);

    // run all fans with minimum duty
    pwm_out.set_duty(Channel::C1, consts::MIN_DUTY);
    pwm_out.set_duty(Channel::C2, consts::MIN_DUTY);
    pwm_out.set_duty(Channel::C3, consts::MIN_DUTY);
    pwm_out.set_duty(Channel::C4, consts::MIN_DUTY);
}

fn get_temperature(adc1_reading: u16) -> f32 {
    let v_out: f32 = adc1_reading as f32 * consts::V_SUPPLY / consts::ADC_RESOLUTION;
    defmt::trace!("v_out {}", v_out);
    defmt::trace!("adc1: {}", adc1_reading);

    let r_ntc = (v_out * consts::R_10K) / (consts::V_SUPPLY - v_out);
    defmt::trace!("rt {}", r_ntc);

    let t_25_c_in_k = consts::ZERO_K_IN_C + 25.0;

    let temp_k = (t_25_c_in_k * consts::B_PARAM)
        / (t_25_c_in_k * (r_ntc / consts::R_10K).ln() + consts::B_PARAM);

    defmt::trace!("k {}", temp_k);
    let c = temp_k - consts::ZERO_K_IN_C;
    defmt::debug!("C {}", c);
    c
}

fn tem_to_duty(temp: f32) -> u16 {
    if temp <= consts::MIN_TEMP {
        0 // stop the fan if tem is really low
    } else if temp >= consts::MAX_TEMP {
        consts::MAX_DUTY
    } else {
        ((consts::MAX_DUTY - consts::MIN_DUTY) as f32 * (temp - consts::MIN_TEMP)
            / (consts::MAX_TEMP - consts::MIN_TEMP)
            + consts::MIN_DUTY as f32) as u16
    }
}

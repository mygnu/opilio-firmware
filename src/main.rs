//! Testing PWM output for pre-defined pin combination: all pins for default
//! mapping

// #![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use micromath::F32Ext;
use stm32f1xx_hal::{
    adc::Adc,
    flash::{FlashSize, SectorSize},
    gpio::{
        gpioa::{PA0, PA1, PA2, PA3},
        Alternate, PushPull,
    },
    pac,
    prelude::*,
    pwm::{self, Channel, Pwm},
    pwm_input::{Configuration, ReadMode},
    time::U32Ext,
    timer::{Tim2NoRemap, Timer},
    usb::UsbBusType,
};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use water_crab::{
    consts,
    models::{FanControl, CONFIG_SIZE},
};

type PwmTimer = Pwm<
    pac::TIM2,
    Tim2NoRemap,
    (pwm::C1, pwm::C2, pwm::C3, pwm::C4),
    (
        PA0<Alternate<PushPull>>,
        PA1<Alternate<PushPull>>,
        PA2<Alternate<PushPull>>,
        PA3<Alternate<PushPull>>,
    ),
>;

static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut dbg = p.DBGMCU;
    let mut writer = flash.writer(SectorSize::Sz1K, FlashSize::Sz64K);

    defmt::println!("CONFIG_SIZE {}", CONFIG_SIZE);

    let mut fan_control = FanControl::default();
    // fan_control.save_to_flash(&mut writer);
    fan_control.print();

    fan_control.load_from_flash(&mut writer);

    fan_control.print();

    let clocks = rcc
        .cfgr
        .adcclk(2.mhz())
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    // Setup ADC
    let mut adc1 = Adc::adc1(p.ADC1, &mut rcc.apb2, clocks);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let gpiob = p.GPIOB.split(&mut rcc.apb2);

    // Configure pb0 as an analog input
    let mut ch0 = gpioa.pa4.into_analog(&mut gpioa.crl);
    // TIM2
    let a0 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let a1 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let a2 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let a3 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);

    let pwm_output_pins = (a0, a1, a2, a3);

    // let mut pwm_timer1: PwmTimer = Timer::tim1(p.TIM1, &clocks, &mut rcc.apb1)
    //     .pwm(pwm_output_pins, &mut afio.mapr, 24.khz());
    let mut pwm_timer2: PwmTimer = Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1)
        .pwm(pwm_output_pins, &mut afio.mapr, 24.khz());
    setup_pwm(&mut pwm_timer2);

    let (_pa15, _pb3, pb4) =
        afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
    let pwm_input_pins = (pb4, gpiob.pb5);
    let pwm_input_timer3 = Timer::tim3(p.TIM3, &clocks, &mut rcc.apb1)
        .pwm_input(
            pwm_input_pins,
            &mut afio.mapr,
            &mut dbg,
            Configuration::Frequency(5.hz()),
        );
    let mut countdown_timer4 =
        Timer::tim4(p.TIM4, &clocks, &mut rcc.apb1).start_count_down(700.ms());

    let max_duty = pwm_timer2.get_max_duty();

    loop {
        nb::block!(countdown_timer4.wait()).unwrap();
        if let Ok(adc1_reading) = adc1.read(&mut ch0) {
            let temp = get_temperature(adc1_reading);

            let duty = fan_control.temp_to_duty(0, temp);
            let actual_duty = max_duty / 100 * duty as u16;
            defmt::println!("{}:{}:{}", duty, max_duty, actual_duty);
            pwm_timer2.set_duty(Channel::C1, actual_duty);
        }

        if let Ok(freq) =
            pwm_input_timer3.read_frequency(ReadMode::Instant, &clocks)
        {
            defmt::trace!("1Htz: {}", freq.0);
            defmt::debug!("1RPM: {}", freq.0 * 30);
        }
    }
}

fn setup_pwm(pwm_timer2: &mut PwmTimer) {
    // Enable clock on each of the channels
    pwm_timer2.enable(Channel::C1);
    pwm_timer2.enable(Channel::C2);
    pwm_timer2.enable(Channel::C3);
    pwm_timer2.enable(Channel::C4);

    let min_duty = pwm_timer2.get_max_duty() * 10 / 100;
    // run all fans with minimum duty
    pwm_timer2.set_duty(Channel::C1, min_duty);
    pwm_timer2.set_duty(Channel::C2, min_duty);
    pwm_timer2.set_duty(Channel::C3, min_duty);
    pwm_timer2.set_duty(Channel::C4, min_duty);
}

fn get_temperature(adc1_reading: u16) -> f32 {
    let v_out: f32 =
        adc1_reading as f32 * consts::V_SUPPLY / consts::ADC_RESOLUTION;
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

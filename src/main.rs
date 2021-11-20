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
        gpioa::{PA0, PA1, PA2, PA3, PA6, PA7},
        gpiob::{PB0, PB1},
        Alternate, PushPull,
    },
    pac,
    prelude::*,
    pwm::{self, Channel, Pwm},
    pwm_input::{Configuration, ReadMode},
    time::U32Ext,
    timer::{Tim2NoRemap, Tim3NoRemap, Timer},
    // usb::UsbBusType,
};
// use usb_device::{bus::UsbBusAllocator, prelude::*};
// use usbd_serial::{SerialPort, USB_CLASS_CDC};
use water_crab::{
    consts,
    models::{FanControl, CONFIG_SIZE},
};

type PwmTimer2 = Pwm<
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

type PwmTimer3 = Pwm<
    pac::TIM3,
    Tim3NoRemap,
    (pwm::C1, pwm::C2, pwm::C3, pwm::C4),
    (
        PA6<Alternate<PushPull>>,
        PA7<Alternate<PushPull>>,
        PB0<Alternate<PushPull>>,
        PB1<Alternate<PushPull>>,
    ),
>;

// static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
// static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
// static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

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

    let mut adc1 = Adc::adc1(p.ADC1, &mut rcc.apb2, clocks);
    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // Configure pa4 as an analog input
    let mut pa4 = gpioa.pa4.into_analog(&mut gpioa.crl);
    let mut countdown_timer1 =
        Timer::tim1(p.TIM1, &clocks, &mut rcc.apb2).start_count_down(700.ms());

    let pins_a0_a3 = (
        gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
    );

    let mut pwm_timer2: PwmTimer2 = Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1)
        .pwm::<Tim2NoRemap, _, _, _>(pins_a0_a3, &mut afio.mapr, 24.khz());
    setup_pwm_t2(&mut pwm_timer2);

    let pins_a6_a9_b0_b1 = (
        gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
        gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl),
        gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl),
    );

    let mut pwm_timer3: PwmTimer3 = Timer::tim3(p.TIM3, &clocks, &mut rcc.apb1)
        .pwm::<Tim3NoRemap, _, _, _>(
            pins_a6_a9_b0_b1,
            &mut afio.mapr,
            24.khz(),
        );
    pwm_timer3.enable(Channel::C1);
    pwm_timer3.set_duty(Channel::C1, 1000);

    setup_pwm_t3(&mut pwm_timer3);

    let pwm_input_pins = (
        gpiob.pb6.into_floating_input(&mut gpiob.crl),
        gpiob.pb7.into_floating_input(&mut gpiob.crl),
    );
    let pwm_input_timer4 = Timer::tim4(p.TIM4, &clocks, &mut rcc.apb1)
        .pwm_input(
            pwm_input_pins,
            &mut afio.mapr,
            &mut dbg,
            Configuration::Frequency(5.hz()),
        );

    let max_duty = pwm_timer2.get_max_duty();

    loop {
        nb::block!(countdown_timer1.wait()).unwrap();
        if let Ok(adc1_reading) = adc1.read(&mut pa4) {
            let temp = get_temperature(adc1_reading);

            let duty = fan_control.temp_to_duty(0, temp);
            let actual_duty = max_duty / 100 * duty as u16;
            defmt::println!("{}:{}:{}", duty, max_duty, actual_duty);
            let current_duty = pwm_timer2.get_duty(Channel::C1);
            if current_duty != actual_duty {
                pwm_timer2.set_duty(Channel::C1, actual_duty);
            }
        }

        if let Ok(freq) =
            pwm_input_timer4.read_frequency(ReadMode::Instant, &clocks)
        {
            defmt::println!("Htz: {}", freq.0);
            defmt::println!("RPM: {}", freq.0 * 30);
        }
    }
}

fn setup_pwm_t2(pwm_timer2: &mut PwmTimer2) {
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

fn setup_pwm_t3(pwm_timer3: &mut PwmTimer3) {
    // Enable clock on each of the channels
    pwm_timer3.enable(Channel::C1);
    pwm_timer3.enable(Channel::C2);
    pwm_timer3.enable(Channel::C3);
    pwm_timer3.enable(Channel::C4);

    let min_duty = pwm_timer3.get_max_duty() * 10 / 100;
    // run all fans with minimum duty
    pwm_timer3.set_duty(Channel::C1, min_duty);
    pwm_timer3.set_duty(Channel::C2, min_duty);
    pwm_timer3.set_duty(Channel::C3, min_duty);
    pwm_timer3.set_duty(Channel::C4, min_duty);
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

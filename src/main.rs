//! Testing PWM output for pre-defined pin combination: all pins for default
//! mapping

// #![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m::asm::delay;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;

use nb::block;
use stm32f1xx_hal::{
    adc::Adc,
    flash::{FlashSize, SectorSize},
    pac::{self, TIM4},
    prelude::*,
    pwm_input::{Configuration, PwmInput, ReadMode},
    time::U32Ext,
    timer::{Tim2NoRemap, Tim3NoRemap, Timer},
    usb::{Peripheral, UsbBus},
};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use opilio::{
    controller::{Controller, CHUNK_SIZE},
    MuxController, MuxInput, PwmInputTimer, PwmTimer2, PwmTimer3,
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut dbg = p.DBGMCU;
    let mut writer = flash.writer(SectorSize::Sz1K, FlashSize::Sz64K);

    defmt::println!("CHUNK_SIZE {}", CHUNK_SIZE);

    // let mut fan_control = Controller::default();
    // fan_control.save_to_flash(&mut writer);
    // fan_control.print();

    // fan_control.load_from_flash(&mut writer);

    // fan_control.print();

    let clocks = rcc
        .cfgr
        .adcclk(2.mhz())
        .use_hse(8.mhz())
        .sysclk(48.mhz()) // USB requires sysclk to be at 48MHz or 72MHz
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);
    assert!(clocks.usbclk_valid());

    let adc1 = Adc::adc1(p.ADC1, &mut rcc.apb2, clocks);
    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // BluePill board has a pull-up resistor on the D+ line.
    // Pull the D+ pin down to send a RESET condition to the USB bus.
    // This forced reset is needed only for development, without it host
    // will not reset your device when you upload new firmware.
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low().ok();
    delay(clocks.sysclk().0 / 100);

    let usb_dm = gpioa.pa11;
    let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

    let usb = Peripheral {
        usb: p.USB,
        pin_dm: usb_dm,
        pin_dp: usb_dp,
    };
    let usb_bus = UsbBus::new(usb);

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev =
        UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0010))
            .manufacturer("LOCAL")
            .product("Water Crab: Open Source PC Fan Controller")
            .serial_number("0xA455")
            .device_class(USB_CLASS_CDC)
            .build();

    // Configure pa4 as an analog input
    let pa4 = gpioa.pa4.into_analog(&mut gpioa.crl);
    let pb12 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    let pb13 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    let pb14 = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);

    let mut mux = MuxController::new(pb12, pb13, pb14);

    mux.enable(MuxInput::L1);

    let mut countdown_timer1 =
        Timer::tim1(p.TIM1, &clocks, &mut rcc.apb2).start_count_down(500.ms());

    let pins_a0_a3 = (
        gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
    );

    let pwm_timer2: PwmTimer2 =
        Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1)
            .pwm::<Tim2NoRemap, _, _, _>(pins_a0_a3, &mut afio.mapr, 24.khz());

    let pins_a6_a9_b0_b1 = (
        gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
        gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl),
        gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl),
    );

    let pwm_timer3: PwmTimer3 = Timer::tim3(p.TIM3, &clocks, &mut rcc.apb1)
        .pwm::<Tim3NoRemap, _, _, _>(
        pins_a6_a9_b0_b1,
        &mut afio.mapr,
        24.khz(),
    );

    let pwm_input_pins = (
        gpiob.pb6.into_floating_input(&mut gpiob.crl),
        gpiob.pb7.into_floating_input(&mut gpiob.crl),
    );
    let pwm_input_timer4: PwmInputTimer =
        Timer::tim4(p.TIM4, &clocks, &mut rcc.apb1).pwm_input(
            pwm_input_pins,
            &mut afio.mapr,
            &mut dbg,
            Configuration::DutyCycle(0.hz()),
        );

    let mut controller = Controller::new(pwm_timer2, pwm_timer3, adc1, pa4);

    loop {
        block!(countdown_timer1.wait()).ok();

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 32];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    defmt::println!("got {:?}", buf);

                    //reply(&buf, &mut serial);
                }
                _ => {}
            }
        }

        controller.adjust_pwm();

        if let Ok(freq) =
            pwm_input_timer4.read_frequency(ReadMode::Instant, &clocks)
        {
            defmt::println!("Htz: {}", freq.0);
            defmt::println!("RPM: {}", freq.0 * 30);
        }
    }
}

pub struct RpmReader {
    mux: MuxController,
    pwm_input_timer: PwmInputTimer,
}

impl RpmReader {
    pub fn new(pwm_input_timer: PwmInputTimer, mux: MuxController) -> Self {
        Self {
            mux,
            pwm_input_timer,
        }
    }
}

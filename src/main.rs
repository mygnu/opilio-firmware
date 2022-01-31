//! Testing PWM output for pre-defined pin combination: all pins for default
//! mapping

// #![deny(unsafe_code)]
#![no_main]
#![no_std]

use core::ops::Deref;

use defmt::debug;
use heapless::Vec;
use opilio::controller::{
    default_configs, Config, BUF_SIZE, CONFIG_SIZE, FLASH_START_OFFSET,
};
use postcard::{from_bytes, to_vec};
use stm32f1xx_hal::flash::{FlashWriter, SZ_1K};
// use stm32f1xx_hal::flash::{FlashWriter, SZ_1K};

// use cortex_m::asm::delay;
// use cortex_m_rt::entry;
// use opilio::{
//     controller::Controller, tacho::TachoReader, MuxController, PwmInputTimer,
//     PwmTimer2, PwmTimer3,
// };
// use stm32f1xx_hal::{
//     adc::Adc,
//     flash::{FlashSize, SectorSize},
//     pac,
//     prelude::*,
//     pwm_input::Configuration,
//     time::U32Ext,
//     timer::{Tim2NoRemap, Tim3NoRemap, Timer},
//     usb::{Peripheral, UsbBus},
// };
// use usb_device::prelude::*;
// use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [RTC], peripherals = true)]
mod app {
    use super::*;
    use cortex_m::asm::delay;
    use heapless::Vec;
    use opilio::{
        controller::{Config, Controller, NO_OF_FANS},
        PwmTimer2, PwmTimer3,
    };
    use stm32f1xx_hal::{
        adc::Adc,
        device::Peripherals,
        flash::{FlashSize, SectorSize},
        pac,
        prelude::*,
        timer::{Tim2NoRemap, Tim3NoRemap, Timer},
    };
    use stm32f1xx_hal::{
        flash::Parts,
        usb::{Peripheral, UsbBus, UsbBusType},
    };
    use systick_monotonic::*;
    use usb_device::prelude::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
        configs: Vec<Config, NO_OF_FANS>,
        flash: Parts,
        controller: Controller,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<
            usb_device::bus::UsbBusAllocator<UsbBusType>,
        > = None;

        let device = cx.device;
        let mut flash = device.FLASH.constrain();
        let rcc = device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .adcclk(8.mhz())
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        let adc1 = Adc::adc1(device.ADC1, clocks);

        let mut gpioa = device.GPIOA.split();
        let mut afio = device.AFIO.constrain();
        let mut gpiob = device.GPIOB.split();

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        unsafe {
            USB_BUS.replace(UsbBus::new(usb));
        }

        let serial =
            usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x1209, 0x0010),
        )
        .manufacturer("Opilio")
        .product("Open Source PC Fan Controller")
        .serial_number("0A455")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        // Initialize the monotonic (SysTick rate is 48 MHz)
        let mono = Systick::new(cx.core.SYST, 48_000_000);

        tick::spawn_after(1.secs()).unwrap();

        let writer = flash.writer(SectorSize::Sz1K, FlashSize::Sz64K);
        let configs = super::load_from_flash(&writer);

        // Configure pa4 as an analog input
        let thermistor_pin = gpioa.pa4.into_analog(&mut gpioa.crl);
        // let writer = flash.writer(SectorSize::Sz1K, FlashSize::Sz64K);

        let pins_a0_a3 = (
            gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
        );
        let pwm_timer2: PwmTimer2 = Timer::tim2(device.TIM2, &clocks)
            .pwm::<Tim2NoRemap, _, _, _>(
            pins_a0_a3,
            &mut afio.mapr,
            24.khz(),
        );

        let pins_a6_a9_b0_b1 = (
            gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
            gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl),
            gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl),
        );
        let pwm_timer3: PwmTimer3 = Timer::tim3(device.TIM3, &clocks)
            .pwm::<Tim3NoRemap, _, _, _>(
            pins_a6_a9_b0_b1,
            &mut afio.mapr,
            24.khz(),
        );
        let controller =
            Controller::new(pwm_timer2, pwm_timer3, adc1, thermistor_pin);

        (
            Shared {
                usb_dev,
                serial,
                configs,
                flash,
                controller,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    #[task( shared = [controller, configs])]
    fn tick(cx: tick::Context) {
        debug!("tick");

        (cx.shared.controller, cx.shared.configs).lock(
            |controller, configs| {
                controller.adjust_pwm(configs);
            },
        );

        // cx.shared
        //     .controller
        //     .lock(|c| c.adjust_pwm(&*cx.shared.configs));

        // Periodic ever 1 seconds
        tick::spawn_after(1.secs()).unwrap();
    }

    #[task(binds = USB_HP_CAN_TX, shared = [usb_dev, serial])]
    fn usb_tx(cx: usb_tx::Context) {
        debug!("usb tx");
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            super::usb_poll(usb_dev, serial);
        });
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, serial])]
    fn usb_rx0(cx: usb_rx0::Context) {
        debug!("usb rx");
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            super::usb_poll(usb_dev, serial);
        });
    }
}

fn usb_poll<B: usb_device::bus::UsbBus>(
    usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
    serial: &mut usbd_serial::SerialPort<'static, B>,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf = [0u8; 8];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            debug!("{:?}", buf);
            // Echo back in upper case
            for c in buf[0..count].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }

            serial.write(&buf[0..count]).ok();
        }
        _ => {}
    }
}

pub fn load_from_flash(writer: &FlashWriter) -> Vec<Config, 8> {
    let mut configs = default_configs();

    let bytes = match writer.read(FLASH_START_OFFSET, SZ_1K as usize) {
        Ok(it) => it,
        _ => return configs,
    };
    for c in configs.iter_mut() {
        let range = c.offset_range();
        let chunk = &bytes[c.offset_range()];
        debug!("range {}; chunk: {}", range, chunk);
        if let Ok(config) = from_bytes::<Config>(chunk) {
            debug!("From memory {}", config);
            if config.is_valid() {
                *c = config;
            }
        }
    }

    configs
}

pub fn save_to_flash(writer: &mut FlashWriter, configs: &Vec<Config, 8>) {
    let mut buff: Vec<u8, BUF_SIZE> = Vec::new();

    configs.iter().for_each(|f| {
        let mut ser: Vec<u8, CONFIG_SIZE> = to_vec(f).unwrap();
        debug!("ser {}", ser.deref());
        ser.resize_default(CONFIG_SIZE).unwrap();
        debug!("after resize {}", ser.deref());

        buff.extend(ser.into_iter());

        debug!("buf {}", buff.deref());
    });

    writer.page_erase(FLASH_START_OFFSET).ok();

    writer.write(FLASH_START_OFFSET, buff.deref()).ok();
}

// #[entry]
// fn main() -> ! {
//     let dp = pac::Peripherals::take().unwrap();

//     let mut flash = dp.FLASH.constrain();
//     let rcc = dp.RCC.constrain();
//     let mut gpioa = dp.GPIOA.split();
//     let mut afio = dp.AFIO.constrain();
//     let mut gpiob = dp.GPIOB.split();

//     let clocks = rcc
//         .cfgr
//         .adcclk(8.mhz())
//         .use_hse(8.mhz())
//         .sysclk(48.mhz()) // USB requires sysclk to be at 48MHz or 72MHz
//         .pclk1(24.mhz())
//         .freeze(&mut flash.acr);

//     assert!(clocks.usbclk_valid());

//     let adc1 = Adc::adc1(dp.ADC1, clocks);

//     let pins_a0_a3 = (
//         gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
//         gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
//         gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
//         gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
//     );
//     let pwm_timer2: PwmTimer2 = Timer::tim2(dp.TIM2, &clocks)
//         .pwm::<Tim2NoRemap, _, _, _>(pins_a0_a3, &mut afio.mapr, 24.khz());

//     let pins_a6_a9_b0_b1 = (
//         gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl),
//         gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
//         gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl),
//         gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl),
//     );
//     let pwm_timer3: PwmTimer3 = Timer::tim3(dp.TIM3, &clocks)
//         .pwm::<Tim3NoRemap, _, _, _>(
//             pins_a6_a9_b0_b1,
//             &mut afio.mapr,
//             24.khz(),
//         );

//     let pwm_input_pins = (
//         gpiob.pb6.into_floating_input(&mut gpiob.crl),
//         gpiob.pb7.into_floating_input(&mut gpiob.crl),
//     );
//     let mut dbg = dp.DBGMCU;
//     let pwm_input_timer: PwmInputTimer = Timer::tim4(dp.TIM4, &clocks)
//         .pwm_input(
//             pwm_input_pins,
//             &mut afio.mapr,
//             &mut dbg,
//             Configuration::DutyCycle(1.hz()),
//         );

//     // Configure pa4 as an analog input
//     let thermistor_pin = gpioa.pa4.into_analog(&mut gpioa.crl);
//     let writer = flash.writer(SectorSize::Sz1K, FlashSize::Sz64K);
//     let mut controller =
//         Controller::new(pwm_timer2, pwm_timer3, adc1, thermistor_pin, writer);

//     let pb12 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
//     let pb13 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
//     let pb14 = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
//     let pb15 = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);

//     let mux = MuxController::new(pb12, pb13, pb14);

//     let mut tacho_reader = TachoReader::new(pwm_input_timer, mux, clocks, pb15);

//     // BluePill board has a pull-up resistor on the D+ line.
//     // Pull the D+ pin down to send a RESET condition to the USB bus.
//     // This forced reset is needed only for development, without it host
//     // will not reset your device when you upload new firmware.
//     let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
//     usb_dp.set_low();
//     delay(clocks.sysclk().0 / 100);
//     // block!(countdown_timer1.wait()).ok();

//     let usb = Peripheral {
//         usb: dp.USB,
//         pin_dm: gpioa.pa11,
//         pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
//     };
//     let usb_bus = UsbBus::new(usb);

//     let mut serial = SerialPort::new(&usb_bus);

//     let mut usb_dev =
//         UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0010))
//             .manufacturer("Opilio")
//             .product("Open Source PC Fan Controller")
//             .serial_number("0A455")
//             .device_class(USB_CLASS_CDC)
//             .build();

//     loop {
//         // block!(countdown_timer1.wait()).ok();
//         // delay(clocks.sysclk().0 / 50);

//         // defmt::println!("{}", BUF_SIZE);

//         if usb_dev.poll(&mut [&mut serial]) {
//             let mut buf = [0u8; 64];

//             match serial.read(&mut buf) {
//                 Ok(count) if count > 0 => {
//                     defmt::println!("got {:?}", buf);
//                 }
//                 _ => {}
//             }

//             match serial.write(&[1, 2, 3]) {
//                 Ok(len) if len > 0 => {}
//                 _ => {}
//             }
//         }
//         controller.adjust_pwm();
//         tacho_reader.read_frequencies();
//     }
// }

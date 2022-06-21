#![no_main]
#![no_std]

use stm32f1xx_hal::{
    gpio::{Floating, Input, PB6, PB7, PB8, PB9},
    pac::{Interrupt, RCC, TIM4},
    rcc::{Enable, Reset},
};
const PRESCALER: u16 = 480;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [RTC], peripherals = true)]
mod app {
    use cortex_m::asm::delay;
    use defmt::{debug, trace};
    use opilio::{
        controller::{Controller, FanId},
        tacho::TachoReader,
        Configs, PwmTimer2,
    };
    use stm32f1xx_hal::{
        adc::Adc,
        flash::{FlashExt, Parts},
        prelude::*,
        timer::Tim2NoRemap,
        usb::{Peripheral, UsbBus, UsbBusType},
    };
    use systick_monotonic::{ExtU64, Systick};
    use usb_device::prelude::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
        configs: Configs,
        flash: Parts,
        controller: Controller,
        tacho: TachoReader,
        tick: u8,
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
            .adcclk(8.MHz())
            .use_hse(8.MHz())
            .sysclk(48.MHz())
            .pclk1(24.MHz())
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
        delay(clocks.sysclk().raw() / 10);

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

        let configs = Configs::from_flash(&mut flash);

        // let pins_a8_a11 = (
        //     gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh),
        //     gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
        //     gpioa.pa10.into_alternate_push_pull(&mut gpioa.crh),
        //     // gpioa.pa11.into_alternate_push_pull(&mut gpioa.crh),
        // );

        // let tim1 =
        //     device
        //         .TIM1
        //         .pwm_hz(pins_a8_a11, &mut afio.mapr, 24.kHz(), &clocks);

        // Configure pa4 as an analog input
        let thermistor_pin = gpioa.pa4.into_analog(&mut gpioa.crl);

        let pins_a0_a3 = (
            gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
        );
        let pwm_timer2: PwmTimer2 = device.TIM2.pwm_hz::<Tim2NoRemap, _, _>(
            pins_a0_a3,
            &mut afio.mapr,
            24.kHz(),
            &clocks,
        );

        // let pins_a6_a9_b0_b1 = (
        //     gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl),
        //     gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
        //     gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl),
        //     gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl),
        // );
        // let pwm_timer3: PwmTimer3 = device.TIM3.pwm_hz::<Tim3NoRemap, _, _>(
        //     pins_a6_a9_b0_b1,
        //     &mut afio.mapr,
        //     24.kHz(),
        //     &clocks,
        // );

        let fan_enable = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        let pump_enable = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
        let red_led = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
        let blue_led = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
        let controller = Controller::new(
            pwm_timer2,
            adc1,
            thermistor_pin,
            fan_enable,
            pump_enable,
            red_led,
            blue_led,
        );

        super::timer4_input_setup(gpiob.pb6, gpiob.pb7, gpiob.pb8, gpiob.pb9);

        let tacho = TachoReader::default();

        periodic::spawn().unwrap();
        // rpm::spawn(FanId::F1).unwrap();

        (
            Shared {
                usb_dev,
                serial,
                configs,
                flash,
                controller,
                tacho,
                tick: 0,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    #[task(shared = [controller, configs, tacho, tick])]
    fn periodic(cx: periodic::Context) {
        trace!("periodic");

        (cx.shared.controller, cx.shared.configs, cx.shared.tick).lock(
            |ctl, configs, tick| {
                if *tick > 10 {
                    ctl.standby_mode();
                } else {
                    ctl.adjust_pwm(configs);
                    *tick += 1;
                }
            },
        );

        // Periodic ever 1 seconds
        periodic::spawn_after(ExtU64::secs(1)).unwrap();
    }

    // #[task( shared = [controller, tacho])]
    // fn rpm(mut cx: rpm::Context, fan_id: FanId) {
    //     cx.shared.tacho.lock(|t| {
    //         let freq1 = t.rpm(FanId::F1);
    //         let freq2 = t.rpm(FanId::F2);
    //         let freq3 = t.rpm(FanId::F3);
    //         let freq4 = t.rpm(FanId::F4);

    //         // println!("reading rpm {}", t);
    //         println!("{}, {}, {}, {}", freq1, freq2, freq3, freq4);
    //     });

    //     // let mut tacho = cx.shared.tacho;
    //     // let mut rpml = cx.shared.rpm;
    //     let next_id = fan_id.iter().next().unwrap_or(FanId::F1);

    //     // (&mut tacho, &mut rpml).lock(|t, _r| {
    //     //     // r[fan_id as usize] = t.read_rpm(fan_id);
    //     //     // t.next_reading(next_id);
    //     //     // debug!("{:?}", r.iter());

    //     //     let rpm = t.read_rpm(FanId::F0);
    //     //     debug!("{}", rpm);
    //     // });

    //     // Periodic ever 1 seconds
    //     rpm::spawn_after(ExtU64::secs(1), next_id).unwrap();
    // }

    #[task(binds = USB_HP_CAN_TX, shared = [usb_dev, serial, configs, flash])]
    fn usb_tx(cx: usb_tx::Context) {
        debug!("usb tx");
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;
        let mut configs = cx.shared.configs;
        let mut flash = cx.shared.flash;

        (&mut usb_dev, &mut serial, &mut configs, &mut flash).lock(
            |usb_dev, serial, configs, flash| {
                opilio::serial_handler::usb_poll(
                    usb_dev, serial, configs, flash,
                );
            },
        );
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // Now Wait For Interrupt is used instead of a busy-wait loop
            // to allow MCU to sleep between interrupts
            // https://developer.arm.com/documentation/ddi0406/c/Application-Level-Architecture/Instruction-Details/Alphabetical-list-of-instructions/WFI
            rtic::export::nop()
        }
    }

    #[task(binds = TIM4, shared = [tacho])]
    fn tim4(mut cx: tim4::Context) {
        let tim = unsafe { &(*super::TIM4::ptr()) };

        cx.shared.tacho.lock(|t| {
            let status_register = tim.sr.read();
            if status_register.cc1if().bits() {
                let current = tim.ccr1.read().bits() as u16;
                t.update(FanId::F1, current);
                // println!("tick1 {:#?}", t);
                tim.sr.write(|w| w.cc1if().clear_bit());
            } else if status_register.cc2if().bits() {
                let current = tim.ccr2.read().bits() as u16;
                t.update(FanId::F2, current);
                // println!("tick2 {:?}", t);
                tim.sr.write(|w| w.cc2if().clear_bit());
            } else if status_register.cc3if().bits() {
                let current = tim.ccr3.read().bits() as u16;
                t.update(FanId::F3, current);
                // println!("tick3 {:?}", t);
                tim.sr.write(|w| w.cc3if().clear_bit());
            } else if status_register.cc4if().bits() {
                let current = tim.ccr4.read().bits() as u16;
                t.update(FanId::F4, current);
                tim.sr.write(|w| w.cc4if().clear_bit());
                // println!("tick4 {:?}", t);
            }
        });
        tim.sr.write(|w| w.uif().clear_bit());
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, serial, configs, flash, tick])]
    fn usb_rx0(cx: usb_rx0::Context) {
        debug!("usb rx");
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;
        let mut configs = cx.shared.configs;
        let mut flash = cx.shared.flash;
        let mut tick = cx.shared.tick;

        // reset tick on usb communication
        tick.lock(|c| {
            *c = 0;
        });

        (&mut usb_dev, &mut serial, &mut configs, &mut flash).lock(
            |usb_dev, serial, configs, flash| {
                opilio::serial_handler::usb_poll(
                    usb_dev, serial, configs, flash,
                );
            },
        );
    }
}

/// setup timer
fn timer4_input_setup(
    _pb6: PB6<Input<Floating>>,
    _pb7: PB7<Input<Floating>>,
    _pb8: PB8<Input<Floating>>,
    _pb9: PB9<Input<Floating>>,
) {
    let tim = unsafe { &(*TIM4::ptr()) };

    unsafe {
        //NOTE(unsafe) this reference will only be used for atomic writes with
        // no side effects
        let rcc = &(*RCC::ptr());
        // Enable and reset the timer peripheral
        TIM4::enable(rcc);
        TIM4::reset(rcc);
    }

    // PB6, PB7, PB8, PB9
    // Disable capture on both channels during setting
    // (for Channel X bit is CCXE)
    tim.ccer.modify(|_, w| {
        w.cc1e()
            .clear_bit()
            .cc2e()
            .clear_bit()
            .cc3e()
            .clear_bit()
            .cc4e()
            .clear_bit()
            .cc1p()
            .set_bit()
            .cc2p()
            .set_bit()
            .cc3p()
            .set_bit()
            .cc4p()
            .set_bit()
    });

    // Define the direction of the channel (input/output)
    // and the used input
    tim.ccmr1_input().modify(|_, w| w.cc1s().ti1().cc2s().ti2());
    tim.ccmr2_input().modify(|_, w| w.cc3s().ti3().cc4s().ti4());

    // DMA enable interrupt register
    tim.dier.write(|w| {
        w.cc1ie()
            .enabled()
            .cc2ie()
            .enabled()
            .cc3ie()
            .enabled()
            .cc4ie()
            .enabled()
    });

    // Configure slave mode control register
    // Selects the trigger input to be used to synchronize the counter
    // 101: Filtered Timer Input 1 (TI1FP1)
    // ---------------------------------------
    // Slave Mode Selection :
    // Trigger Mode - The counter starts at a rising edge of the trigger TRGI
    // (but it is not reset). Only the start of the counter is controlled.
    tim.smcr.modify(|_, w| w.ts().ti1fp1().sms().trigger_mode());

    // auto reload register
    tim.arr.write(|w| w.arr().bits(u16::MAX));
    tim.psc.write(|w| w.psc().bits(PRESCALER));
    tim.ccer.modify(|_, w| {
        w.cc1e()
            .set_bit()
            .cc2e()
            .set_bit()
            .cc3e()
            .set_bit()
            .cc4e()
            .set_bit()
    });
    tim.cr1.modify(|_, w| w.cen().set_bit());
    unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::TIM4) };
}

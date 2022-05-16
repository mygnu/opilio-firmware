#![no_main]
#![no_std]

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [RTC], peripherals = true)]
mod app {
    use cortex_m::asm::delay;
    use defmt::{debug, trace};
    use heapless::Vec;
    use opilio::{
        controller::{default_rpm, Controller, FanId},
        tacho::TachoReader,
        Configs, MuxController, PwmInputTimer, PwmTimer2, PwmTimer3,
    };
    use stm32f1xx_hal::{
        adc::Adc,
        flash::{FlashExt, Parts},
        prelude::*,
        timer::{pwm_input::Configuration, Tim2NoRemap, Tim3NoRemap, Timer},
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
        rpm: Vec<u32, 8>,
        controller: Controller,
        tacho: TachoReader,
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
        delay(clocks.sysclk().raw());

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

        let pins_a6_a9_b0_b1 = (
            gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
            gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl),
            gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl),
        );
        let pwm_timer3: PwmTimer3 = device.TIM3.pwm_hz::<Tim3NoRemap, _, _>(
            pins_a6_a9_b0_b1,
            &mut afio.mapr,
            24.kHz(),
            &clocks,
        );
        let controller =
            Controller::new(pwm_timer2, pwm_timer3, adc1, thermistor_pin);

        let pb12 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        let pb13 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
        let pb14 = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
        let pb15 = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);

        let mux = MuxController::new(pb12, pb13, pb14);

        let pwm_input_pins = (
            gpiob.pb6.into_floating_input(&mut gpiob.crl),
            gpiob.pb7.into_floating_input(&mut gpiob.crl),
        );
        let mut dbg = device.DBGMCU;
        let pwm_input_timer: PwmInputTimer = Timer::new(device.TIM4, &clocks)
            .pwm_input(
                pwm_input_pins,
                &mut afio.mapr,
                &mut dbg,
                Configuration::Frequency(1.Hz()),
            );

        let tacho = TachoReader::new(pwm_input_timer, mux, clocks, pb15);

        tick::spawn().unwrap();
        rpm::spawn(FanId::F0).unwrap();

        (
            Shared {
                rpm: default_rpm(),
                usb_dev,
                serial,
                configs,
                flash,
                controller,
                tacho,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    #[task( shared = [controller, configs, tacho])]
    fn tick(cx: tick::Context) {
        trace!("tick");

        (cx.shared.controller, cx.shared.configs).lock(
            |controller, configs| {
                controller.adjust_pwm(configs);
            },
        );

        // Periodic ever 1 seconds
        tick::spawn_after(ExtU64::secs(1)).unwrap();
    }

    #[task( shared = [controller, rpm, tacho])]
    fn rpm(cx: rpm::Context, fan_id: FanId) {
        debug!("rpm");

        let mut tacho = cx.shared.tacho;
        let mut rpml = cx.shared.rpm;
        let next_id = fan_id.next_id();

        (&mut tacho, &mut rpml).lock(|t, r| {
            r[fan_id as usize] = t.read_rpm(fan_id);
            t.next_reading(next_id);
            debug!("{:?}", r.iter());
        });

        // Periodic ever 1 seconds
        rpm::spawn_after(ExtU64::secs(1), next_id).unwrap();
    }

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

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, serial, configs, flash])]
    fn usb_rx0(cx: usb_rx0::Context) {
        // debug!("usb rx");
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
}

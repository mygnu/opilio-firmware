#![no_main]
#![no_std]

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [RTC], peripherals = true)]
mod app {

    use cortex_m::asm::delay;
    use defmt::{debug, trace};
    use opilio_firmware::{
        controller::{Controller, RgbLed, Temps, TICK_PERIOD},
        serial_handler::UsbHandler,
        tacho::TachoReader,
        FlashOps, PwmTimer2,
    };
    use opilio_lib::{Config, Id, PID, VID};
    use stm32f1xx_hal::{
        adc::Adc,
        flash::{FlashExt, Parts},
        pac::TIM4,
        prelude::*,
        timer::Tim2NoRemap,
        usb::{Peripheral, UsbBus, UsbBusType},
    };
    use systick_monotonic::{ExtU64, Systick};
    use usb_device::prelude::*;

    const VERSION: &str = "1.2.0";

    use super::timer_setup::timer4_input_setup;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[shared]
    struct Shared {
        config: Config,
        controller: Controller,
        tacho: TachoReader,
        tick: u64,
    }

    #[local]
    struct Local {
        usb_handler: UsbHandler<UsbBusType>,
        flash: Parts,
    }

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
            UsbVidPid(VID, PID),
        )
        .manufacturer("Open Hardware")
        .product("Opilio - PC Fan/Pump Controller")
        .serial_number(VERSION)
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        let usb_handler = UsbHandler::new(usb_dev, serial);

        // Initialize the monotonic (SysTick rate is 48 MHz)
        let mono = Systick::new(cx.core.SYST, 48_000_000);

        // Configure pa4 as an analog input
        let ambient_thermistor = gpioa.pa4.into_analog(&mut gpioa.crl);
        let coolant_thermistor = gpioa.pa5.into_analog(&mut gpioa.crl);
        let coolant_out_thermistor = gpioa.pa6.into_analog(&mut gpioa.crl);

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

        let buzzer = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);
        let pump_power = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        let fan_power = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
        let red_led = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
        let green_led = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
        let blue_led = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
        let mut led = RgbLed::new(red_led, green_led, blue_led);
        led.exclusive_on(opilio_firmware::controller::Led::GREEN);

        let config = if let Ok(config) = Config::from_flash(&mut flash) {
            if config.is_valid() {
                defmt::debug!("Config from disk: {:?}", config);
                config
            } else {
                defmt::error!("Invalid {:?}", config);
                Config::default()
            }
        } else {
            let config = Config::default();
            defmt::error!("Failed to create, using Default {}", config);
            config
        };

        let temps = Temps::new(
            adc1,
            ambient_thermistor,
            coolant_thermistor,
            coolant_out_thermistor,
        );

        let controller = Controller::new(
            pwm_timer2, pump_power, fan_power, buzzer, temps, led,
        );

        timer4_input_setup(gpiob.pb6, gpiob.pb7, gpiob.pb8, gpiob.pb9);

        let tacho = TachoReader::default();

        periodic::spawn().ok();

        (
            Shared {
                config,
                controller,
                tacho,
                tick: 0,
            },
            Local { usb_handler, flash },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [controller, config, tacho, tick])]
    fn periodic(cx: periodic::Context) {
        trace!("periodic");

        (cx.shared.controller, cx.shared.config, cx.shared.tick).lock(
            |ctl, config, tick| {
                if *tick > config.general.sleep_after as u64 * 1000 {
                    ctl.adjust_pwm(config, true);
                } else {
                    ctl.adjust_pwm(config, false);
                    *tick = tick.saturating_add(TICK_PERIOD);
                }
            },
        );

        periodic::spawn_after(TICK_PERIOD.millis()).ok();
    }

    // #[task(binds = USB_HP_CAN_TX, shared = [])]
    // fn usb_tx(_cx: usb_tx::Context) {
    //     debug!("usb tx");
    // }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // defmt::info!("idle");
            rtic::export::nop();
        }
    }

    /// timer4 interrupt handler
    /// triggers every time a channel is on the rising edge
    /// updates the tacho value to be
    #[task(binds = TIM4, shared = [tacho])]
    fn tim4(mut cx: tim4::Context) {
        let tim = unsafe { &(*TIM4::ptr()) };

        cx.shared.tacho.lock(|t| {
            let status_register = tim.sr.read();
            if status_register.cc1if().bit_is_set() {
                let current = tim.ccr1().read().bits() as u16;
                t.update(Id::P1, current);
                tim.sr.write(|w| w.cc1if().clear_bit());
            } else if status_register.cc2if().bit_is_set() {
                let current = tim.ccr2().read().bits() as u16;
                t.update(Id::F1, current);

                tim.sr.write(|w| w.cc2if().clear_bit());
            } else if status_register.cc3if().bit_is_set() {
                let current = tim.ccr3().read().bits() as u16;
                t.update(Id::F2, current);

                tim.sr.write(|w| w.cc3if().clear_bit());
            } else if status_register.cc4if().bit_is_set() {
                let current = tim.ccr4().read().bits() as u16;
                t.update(Id::F3, current);
                tim.sr.write(|w| w.cc4if().clear_bit());
            }
        });
        tim.sr.write(|w| w.uif().clear_bit());
    }

    /// usb_rx0 interrupt handler
    /// triggers every time there is incoming data on usb serial bus
    #[task(binds = USB_LP_CAN_RX0, local = [usb_handler, flash], shared = [ config,  tick, controller, tacho])]
    fn usb_rx0(cx: usb_rx0::Context) {
        debug!("usb rx");
        let usb_handler = cx.local.usb_handler;
        let mut config = cx.shared.config;
        let flash = cx.local.flash;
        let mut tick = cx.shared.tick;
        let mut controller = cx.shared.controller;
        let mut tacho = cx.shared.tacho;

        // reset tick on usb communication
        tick.lock(|c| {
            *c = 0;
        });

        (&mut config, &mut controller, &mut tacho).lock(
            |config, controller, tacho| {
                usb_handler.poll(config, flash, controller, tacho);
            },
        );
    }
}

mod timer_setup {
    use stm32f1xx_hal::{
        gpio::{Floating, Input, PB6, PB7, PB8, PB9},
        pac::{Interrupt, RCC, TIM4},
        rcc::{Enable, Reset},
    };

    /// since main clock is 48Mhz,
    /// 480 prescaler gives us a nice 10 microsecond tick
    /// good enough to measure fan rpm values
    pub const PRESCALER: u16 = 480;

    /// setup timer (fake consumption of pins so we don't accidentally use them
    /// for other purposes)
    pub fn timer4_input_setup(
        _pb6: PB6<Input<Floating>>,
        _pb7: PB7<Input<Floating>>,
        _pb8: PB8<Input<Floating>>,
        _pb9: PB9<Input<Floating>>,
    ) {
        let tim = unsafe { &(*TIM4::ptr()) };

        unsafe {
            //NOTE(unsafe) this reference will only be used for atomic writes
            // with no side effects
            let rcc = &(*RCC::ptr());
            // Enable and reset the timer peripheral
            TIM4::enable(rcc);
            TIM4::reset(rcc);
        }

        // PB6, PB7, PB8, PB9
        // Disable capture on all 4 channels during setting
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

        // configure capture/compare mode resistors and map Timer Input
        // registers
        tim.ccmr1_input().modify(|_, w| w.cc1s().ti1().cc2s().ti2());
        tim.ccmr2_input().modify(|_, w| w.cc3s().ti3().cc4s().ti4());

        // configure DMA enable interrupt register for all 4 channels
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
        // Trigger Mode - The counter starts at a rising edge of the trigger
        // TRGI (but it is not reset). Only the start of the counter is
        // controlled.
        tim.smcr.modify(|_, w| w.ts().ti1fp1().sms().trigger_mode());

        // set auto reload register and prescaler
        tim.arr.write(|w| w.arr().bits(u16::MAX));
        tim.psc.write(|w| w.psc().bits(PRESCALER));

        // enable capture/compare
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

        // enable timer counter
        tim.cr1.modify(|_, w| w.cen().set_bit());

        // setup timer interrupt
        unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::TIM4) };
    }
}

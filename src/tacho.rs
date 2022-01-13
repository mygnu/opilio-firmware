use cortex_m::asm::delay;
use stm32f1xx_hal::{
    gpio::{gpiob::PB15, Output, PushPull},
    pwm_input::ReadMode,
    rcc::Clocks,
};

use crate::{controller::FanId, MuxController, PwmInputTimer};

pub struct TachoReader {
    mux: MuxController,
    enable_pin: PB15<Output<PushPull>>,
    pwm_input_timer: PwmInputTimer,
    clocks: Clocks,
}

impl TachoReader {
    pub fn new(
        pwm_input_timer: PwmInputTimer,
        mux: MuxController,
        clocks: Clocks,
        enable_pin: PB15<Output<PushPull>>,
    ) -> Self {
        Self {
            mux,
            pwm_input_timer,
            clocks,
            enable_pin,
        }
    }

    pub fn read_frequencies(&mut self) {
        use FanId::*;
        for mode in [F0, F1, F2, F3, F4, F5, F6, F7] {
            self.enable_pin.set_high();
            delay(self.clocks.sysclk().0 / 100);
            self.enable_pin.set_low();
            self.mux.enable(mode);
            delay(self.clocks.sysclk().0 / 5);
            if let Ok(freq) = self
                .pwm_input_timer
                .read_frequency(ReadMode::Instant, &self.clocks)
            {
                defmt::info!("Htz: {}", freq.0);
                defmt::info!("{} RPM: {}", mode, freq.0 * 30);
            }
        }
    }
}

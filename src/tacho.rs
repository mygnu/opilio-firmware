use stm32f1xx_hal::{pwm_input::ReadMode, rcc::Clocks};

use crate::{controller::FanId, MuxController, PwmInputTimer};

pub struct TachoReader {
    mux: MuxController,
    pwm_input_timer: PwmInputTimer,
    clocks: Clocks,
}

impl TachoReader {
    pub fn new(
        pwm_input_timer: PwmInputTimer,
        mux: MuxController,
        clocks: Clocks,
    ) -> Self {
        Self {
            mux,
            pwm_input_timer,
            clocks,
        }
    }

    pub fn read_frequencies(&mut self) {
        self.mux.enable(FanId::F0);
        if let Ok(freq) = self
            .pwm_input_timer
            .read_frequency(ReadMode::Instant, &self.clocks)
        {
            defmt::println!("Htz: {}", freq.0);
            defmt::println!("RPM: {}", freq.0 * 30);
        }
    }
}

use cortex_m::asm::delay;
use stm32f1xx_hal::{
    gpio::{gpiob::PB15, Output, PushPull},
    pwm_input::ReadMode,
    rcc::Clocks,
};

use crate::{controller::FanId, MuxController, PwmInputTimer};

pub struct TachoReader {
    mux: MuxController,
    // enable_pin: PB15<Output<PushPull>>,
    pwm_input_timer: PwmInputTimer,
    clocks: Clocks,
}

impl TachoReader {
    pub fn new(
        pwm_input_timer: PwmInputTimer,
        mux: MuxController,
        clocks: Clocks,
        mut enable_pin: PB15<Output<PushPull>>,
    ) -> Self {
        enable_pin.set_low();
        Self {
            mux,
            pwm_input_timer,
            clocks,
            // enable_pin,
        }
    }

    pub fn next_reading(&mut self, fan_id: FanId) {
        self.mux.switch(fan_id);
    }

    pub fn read_rpm(&mut self, fan_id: FanId) -> u32 {
        defmt::info!("Reading {}", fan_id);

        delay(self.clocks.sysclk().0 / 3);
        if let Ok(freq) = self
            .pwm_input_timer
            .read_frequency(ReadMode::Instant, &self.clocks)
        {
            freq.0 * 30
        } else {
            0
        }
    }
}

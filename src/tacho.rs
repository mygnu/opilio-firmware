use defmt::Format;
use shared::FanId;

const FANS: &[FanId] = &[FanId::F1, FanId::F2, FanId::F3, FanId::F4];

#[derive(Default, Format)]
struct Tick {
    previous_ticks: u16,
    elapsed_ticks: u16,
    updates: u8,
}
impl Tick {
    #[inline(always)]
    fn update(&mut self, update: u8) {
        if self.updates > 30 {
            self.elapsed_ticks = 0;
            self.previous_ticks = 0;
        } else {
            self.updates = self.updates.saturating_add(update);
        }
    }
}

#[derive(Default, Format)]
pub struct TachoReader {
    f1: Tick,
    f2: Tick,
    f3: Tick,
    f4: Tick,
}

impl TachoReader {
    #[inline(always)]
    fn get_tick(&mut self, fan_id: &FanId) -> &mut Tick {
        match fan_id {
            FanId::F1 => &mut self.f1,
            FanId::F2 => &mut self.f2,
            FanId::F3 => &mut self.f3,
            FanId::F4 => &mut self.f4,
        }
    }

    #[inline(always)]
    pub fn update(&mut self, fan_id: FanId, current: u16) {
        let t = self.get_tick(&fan_id);

        defmt::trace!("{}, {}, {}", fan_id, current, t);

        if current > t.previous_ticks {
            t.elapsed_ticks = current - t.previous_ticks;
        } else {
            t.elapsed_ticks = (u16::MAX as u32 + current as u32
                - t.previous_ticks as u32) as u16;
        }
        t.updates = 0;
        t.previous_ticks = current;

        for fan in FANS.iter().filter(|fid| *fid != &fan_id) {
            self.get_tick(&fan).update(1)
        }
    }

    pub fn rpm(&mut self, fan_id: FanId) -> f32 {
        let t = self.get_tick(&fan_id);

        if t.elapsed_ticks == 0 {
            0.0
        } else {
            let rpm = 48_000_000.0 / (481.0 * t.elapsed_ticks as f32) * 30.0;
            t.update(15);
            rpm
        }
    }
    pub fn rpm_data(&mut self) -> (f32, f32, f32, f32) {
        (
            self.rpm(FanId::F1),
            self.rpm(FanId::F2),
            self.rpm(FanId::F3),
            self.rpm(FanId::F4),
        )
    }
}

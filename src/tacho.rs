use defmt::Format;
use opilio_lib::Id;

const FANS: &[Id] = &[Id::P1, Id::F1, Id::F2, Id::F3];

#[derive(Default, Format)]
struct Tick {
    previous_ticks: u16,
    elapsed_ticks: u16,
    updates: u8,
}
impl Tick {
    // reset ticks if no data is received for a while
    #[inline(always)]
    fn tick(&mut self, update: u8) {
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
    p1: Tick,
    f1: Tick,
    f2: Tick,
    f3: Tick,
}

impl TachoReader {
    #[inline(always)]
    fn get_tick(&mut self, fan_id: &Id) -> &mut Tick {
        match fan_id {
            Id::P1 => &mut self.p1,
            Id::F1 => &mut self.f1,
            Id::F2 => &mut self.f2,
            Id::F3 => &mut self.f3,
        }
    }

    #[inline(always)]
    pub fn update(&mut self, fan_id: Id, current: u16) {
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

        // register one tick for the rest
        for fan in FANS.iter().filter(|fid| *fid != &fan_id) {
            self.get_tick(&fan).tick(1)
        }
    }

    #[inline(always)]
    pub fn rpm_data(&mut self) -> (f32, f32, f32, f32) {
        let rpm = |tr: &mut TachoReader, fan_id: Id| -> f32 {
            let t = tr.get_tick(&fan_id);

            if t.elapsed_ticks == 0 {
                0.0
            } else {
                let rpm =
                    48_000_000.0 / (481.0 * t.elapsed_ticks as f32) * 30.0;
                t.tick(15);
                rpm
            }
        };
        (
            rpm(self, Id::P1),
            rpm(self, Id::F1),
            rpm(self, Id::F2),
            rpm(self, Id::F3),
        )
    }
}

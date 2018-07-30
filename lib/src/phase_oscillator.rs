use std::fmt;
use std::cell;

pub struct PhaseOscillator {
    current_phase: cell::Cell<f64>,
}

impl PhaseOscillator {
    pub fn new() -> Self {
        PhaseOscillator { current_phase: cell::Cell::new(0.) }
    }

    pub fn get_phase(&self) -> f64 {
        self.current_phase.get()
    }

    pub fn set_phase(&self, phase: f64) {
        self.current_phase.set(phase);
    }

    pub fn replace_phase(&self, phase: f64) -> f64 {
        self.current_phase.replace(phase)
    }

    pub fn step(&self, phase_speed: f64, time_delta: f64) -> f64 {
        // proceed one step and return new updated phase
        let updated_phase = self.current_phase.get() + phase_speed * time_delta;
        self.current_phase.set(updated_phase);
        updated_phase
    }
}

impl fmt::Display for PhaseOscillator {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "PhaseOscillator",)
    }
}

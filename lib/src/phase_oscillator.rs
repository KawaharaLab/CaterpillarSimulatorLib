use std::fmt;

#[derive(Copy, Clone)]
pub struct PhaseOscillator {
    current_phase: f64,
}

impl PhaseOscillator {
    pub fn new() -> Self {
        PhaseOscillator { current_phase: 0. }
    }

    pub fn get_phase(&self) -> f64 {
        self.current_phase
    }

    pub fn step(&mut self, phase_speed: f64, time_delta: f64) -> f64 {
        // proceed one step and return new updated phase
        self.current_phase += phase_speed * time_delta;
        self.current_phase
    }
}

impl fmt::Display for PhaseOscillator {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "PhaseOscillator",)
    }
}

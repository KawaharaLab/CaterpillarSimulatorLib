use std::fmt;

pub struct Config {
    pub time_delta: f64,
    pub somite_mass: f64,
    pub somite_radius: f64,
    pub normal_angular_velocity: f64,
    pub rts_max_natural_length: f64,
    pub rts_k: f64,
    pub rts_c: f64,
    pub rts_amp: f64,
    pub sp_natural_length: f64,
    pub sp_k: f64,
    pub dp_c: f64,
    pub horizon_ts_k: f64,
    pub vertical_ts_k: f64,
    pub realtime_tunable_ts_rom: f64,
    pub friction_coeff: f64,
}

impl fmt::Display for Config {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "Caterpillar Config\n\
             somite mass: {} kg\n\
             somite radius: {} m\n\
             friction coefficient: {} Ns/m\n\
             [rts]\n\
             k: {} N/m\n\
             c: {} Ns/m\n\
             amp: {} m\n\
             max natural length: {} m\n\
             normal angular verocity: {} rad/s\n\
             [spring]\n\
             k: {} N/m\n\
             natural_length: {} m\n\
             [dumper]\n\
             c: {} Ns/m\n\
             [torsion spring]\n\
             horizon k: {} N/rad\n\
             vertical k: {} N/rad\n\
             [realtime tunable torsion spring]
             range of motion: {} rad\n\
             [simulation]\n\
             one time step: {} s",
            self.somite_mass,
            self.somite_radius,
            self.friction_coeff,
            self.rts_k,
            self.rts_c,
            self.rts_amp,
            self.rts_max_natural_length,
            self.normal_angular_velocity,
            self.sp_k,
            self.sp_natural_length,
            self.dp_c,
            self.horizon_ts_k,
            self.vertical_ts_k,
            self.realtime_tunable_ts_rom,
            self.time_delta,
        )
    }
}

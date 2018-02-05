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
    pub friction_coeff: f64,
}

impl fmt::Display for Config {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "Caterpillar Config\n\
             somite mass: {}\n\
             somite radius: {}\n\
             friction coefficient: {}\n\
             [rts]\n\
             k: {}\n\
             c: {}\n\
             amp: {}\n\
             max natural length: {}\n\
             normal angular verocity: {}\n\
             [spring]\n\
             k: {}\n\
             natural_length: {}\n\
             [dumper]\n\
             c: {}\n\
             [simulation]\n\
             one time step: {}sec",
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
            self.time_delta
        )
    }
}

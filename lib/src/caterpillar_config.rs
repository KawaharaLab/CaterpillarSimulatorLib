use std::fmt;
use std::default;
use std::f64;

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

impl Config {
    pub fn new() -> Self {
        Config::default()
    }

    pub fn set(&mut self, key: &str, val: f64) {
        match key {
            "somite_mass" => self.somite_mass = val,
            "somite_radius" => self.somite_radius = val,
            "normal_angular_velocity" => self.normal_angular_velocity = val,
            "rts_max_natural_length" => self.rts_max_natural_length = val,
            "rts_k" => self.rts_k = val,
            "rts_c" => self.rts_c = val,
            "rts_amp" => self.rts_amp = val,
            "sp_natural_length" => self.sp_natural_length = val,
            "sp_k" => self.sp_k = val,
            "dp_c" => self.dp_c = val,
            "horizon_ts_k" => self.horizon_ts_k = val,
            "vertical_ts_k" => self.vertical_ts_k = val,
            "realtime_tunable_ts_rom" => self.realtime_tunable_ts_rom = val,
            "friction_coeff" => self.friction_coeff = val,
            _ => {}
        };
    }
}

impl default::Default for Config {
    fn default() -> Self {
        Config {
            time_delta: 0.01,                               // s
            somite_mass: 1.,                                // kg
            somite_radius: 0.1,                             // m
            normal_angular_velocity: f64::consts::PI,       // rad/s
            rts_max_natural_length: 0.1,                    // m
            rts_k: 100.,                                    // N/m
            rts_c: 10.,                                     // Ns/m
            rts_amp: 0.05,                                  // m
            sp_natural_length: 0.1,                         // m
            sp_k: 100.,                                     // N/m
            dp_c: 10.,                                      // Ns/m
            horizon_ts_k: 100.,                             // N/m
            vertical_ts_k: 100.,                            // N/m
            realtime_tunable_ts_rom: 0.5 * f64::consts::PI, // rad
            friction_coeff: 10.,                            // Ns/m
        }
    }
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

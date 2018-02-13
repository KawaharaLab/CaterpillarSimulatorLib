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
    pub horizon_ts_k0: f64,
    pub horizon_ts_k1: f64,
    pub vertical_ts_k0: f64,
    pub vertical_ts_k1: f64,
    pub vertical_realtime_tunable_torsion_spirng_k: f64,
    pub realtime_tunable_ts_rom: f64,
    pub static_friction_coeff: f64,
    pub dynamic_friction_coeff: f64,
    pub viscosity_friction_coeff: f64,
    pub tip_sub_static_friction_coeff: f64,
    pub tip_sub_dynamic_friction_coeff: f64,
    pub tip_sub_viscosity_friction_coeff: f64,
    pub friction_switch_tan: f64,
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
            "horizon_ts_k0" => self.horizon_ts_k0 = val,
            "horizon_ts_k1" => self.horizon_ts_k1 = val,
            "vertical_ts_k0" => self.vertical_ts_k0 = val,
            "vertical_ts_k1" => self.vertical_ts_k1 = val,
            "vertical_realtime_tunable_torsion_spirng_k" => {
                self.vertical_realtime_tunable_torsion_spirng_k = val
            }
            "realtime_tunable_ts_rom" => self.realtime_tunable_ts_rom = val,
            "static_friction_coeff" => self.static_friction_coeff = val,
            "dynamic_friction_coeff" => self.dynamic_friction_coeff = val,
            "viscosity_friction_coeff" => self.viscosity_friction_coeff = val,
            "tip_sub_static_friction_coeff" => self.tip_sub_static_friction_coeff = val,
            "tip_sub_dynamic_friction_coeff" => self.tip_sub_dynamic_friction_coeff = val,
            "tip_sub_viscosity_friction_coeff" => self.tip_sub_viscosity_friction_coeff = val,
            "friction_switch_tan" => self.friction_switch_tan = val,
            _ => panic!("invalid config: {}", key),
        };
    }
}

impl default::Default for Config {
    fn default() -> Self {
        Config {
            time_delta: 0.01,                                // s
            somite_mass: 1.,                                 // kg
            somite_radius: 0.1,                              // m
            normal_angular_velocity: f64::consts::PI,        // rad/s
            rts_max_natural_length: 0.1,                     // m
            rts_k: 100.,                                     // N/m
            rts_c: 10.,                                      // Ns/m
            rts_amp: 0.05,                                   // m
            sp_natural_length: 0.1,                          // m
            sp_k: 100.,                                      // N/m
            dp_c: 10.,                                       // Ns/m
            horizon_ts_k0: 0.,                               // N/rad
            horizon_ts_k1: 0.,                               // N/rad
            vertical_ts_k0: 0.,                              // N/rad
            vertical_ts_k1: 0.,                              // N/rad
            vertical_realtime_tunable_torsion_spirng_k: 10., // N/rad
            realtime_tunable_ts_rom: 0.5 * f64::consts::PI,  // rad
            static_friction_coeff: 10.,                      //
            dynamic_friction_coeff: 7.,                      //
            viscosity_friction_coeff: 5.,                    // Ns/m
            tip_sub_static_friction_coeff: 1.,               //
            tip_sub_dynamic_friction_coeff: 0.7,             //
            tip_sub_viscosity_friction_coeff: 0.5,           // Ns/m
            friction_switch_tan: 0.7,                        //
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
             static friction coefficient: {} Ns/m\n\
             dynamic friction coefficient: {} Ns/m\n\
             viscosity friction coefficient: {} \n\
             tip sub static friction coefficient: {} Ns/m\n\
             tip sub dynamic friction coefficient: {} Ns/m\n\
             tip sub viscosity friction coefficient: {} \n\
             friction switch tan: {} \n\
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
             horizon k0: {} N/rad\n\
             horizon k1: {} N/rad\n\
             vertical k0: {} N/rad\n\
             vertical k1: {} N/rad\n\
             vertical_realtime_tunable_torsion_spirng_k: {} N/rad\n\
             [realtime tunable torsion spring]
             range of motion: {} rad\n\
             [simulation]\n\
             one time step: {} s",
            self.somite_mass,
            self.somite_radius,
            self.static_friction_coeff,
            self.dynamic_friction_coeff,
            self.viscosity_friction_coeff,
            self.tip_sub_static_friction_coeff,
            self.tip_sub_dynamic_friction_coeff,
            self.tip_sub_viscosity_friction_coeff,
            self.friction_switch_tan,
            self.rts_k,
            self.rts_c,
            self.rts_amp,
            self.rts_max_natural_length,
            self.normal_angular_velocity,
            self.sp_k,
            self.sp_natural_length,
            self.dp_c,
            self.horizon_ts_k0,
            self.horizon_ts_k1,
            self.vertical_ts_k0,
            self.vertical_ts_k1,
            self.vertical_realtime_tunable_torsion_spirng_k,
            self.realtime_tunable_ts_rom,
            self.time_delta,
        )
    }
}

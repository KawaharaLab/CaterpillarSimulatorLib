pub fn differentiate(previous_val: f64, current_val: f64, dt: f64) -> Option<f64> {
    if dt == 0. {
        None
    } else {
        Some((current_val - previous_val) / dt)
    }
}

pub fn hysteresis_function(v: f64, dv: f64) -> f64 {
    // hysteresis loop between (0, c_{min}) and (\theta_{max}, 1)
    // interpolation with parabola
    let c_min = 0.2_f64;
    let c_max = 1.0_f64;
    if dv >= 0. {
        c_min
    } else {
        c_max
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_hysteresis_function() {
        assert_eq!(hysteresis_function(1., 0.5), 0.2_f64);
        assert_eq!(hysteresis_function(1., -0.3), 1.0_f64);
    }
}

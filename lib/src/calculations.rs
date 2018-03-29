pub fn differentiate(previous_val: f64, current_val: f64, dt: f64) -> Option<f64> {
    if dt == 0. {
        None
    } else {
        Some((current_val - previous_val) / dt)
    }
}

#[cfg(test)]
mod test {}

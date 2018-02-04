use std::fmt;

#[derive(Copy, Clone)]
pub struct RTS {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl fmt::Display for RTS {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "RTS position ({}, {}, {})", self.x, self.y, self.z)
    }
}

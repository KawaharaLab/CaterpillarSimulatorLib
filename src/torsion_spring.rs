use std::fmt;

#[derive(Copy, Clone)]
pub struct TSP {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl fmt::Display for TSP {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Somite position ({}, {}, {})", self.x, self.y, self.z)
    }
}

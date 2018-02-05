use std::fmt;
use std::ops;

#[derive(Copy, Clone)]
pub struct Coordinate {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Coordinate {
    pub fn zero() -> Self {
        Coordinate {
            x: 0.,
            y: 0.,
            z: 0.,
        }
    }

    pub fn to_tuple(&self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }

    pub fn from_tuple(t: (f64, f64, f64)) -> Self {
        Coordinate {
            x: t.0,
            y: t.1,
            z: t.2,
        }
    }

    pub fn norm(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }
}

impl ops::Add for Coordinate {
    type Output = Coordinate;
    fn add(self, rhs: Self) -> Self {
        Coordinate {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl ops::AddAssign for Coordinate {
    fn add_assign(&mut self, rhs: Self) {
        *self = Coordinate {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

impl ops::Sub for Coordinate {
    type Output = Coordinate;
    fn sub(self, rhs: Self) -> Self {
        Coordinate {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl ops::SubAssign for Coordinate {
    fn sub_assign(&mut self, rhs: Self) {
        *self = Coordinate {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        };
    }
}

impl ops::Mul<f64> for Coordinate {
    type Output = Coordinate;
    fn mul(self, rhs: f64) -> Self {
        Coordinate {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl ops::Div<f64> for Coordinate {
    type Output = Coordinate;
    fn div(self, rhs: f64) -> Self {
        Coordinate {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl fmt::Display for Coordinate {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "(x:{}, y:{}, z:{})", self.x, self.y, self.z)
    }
}

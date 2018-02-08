use std::fmt;
use std::ops;

#[derive(Copy, Clone, Debug)]
pub struct Coordinate {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Coordinate {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Coordinate { x: x, y: y, z: z }
    }

    pub fn zero() -> Self {
        Self::new(0., 0., 0.)
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

    pub fn inner_product(&self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn cross_product(&self, rhs: Self) -> Self {
        Coordinate {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
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

impl PartialEq for Coordinate {
    fn eq(&self, rhs: &Self) -> bool {
        self.x == rhs.x && self.y == rhs.y && self.z == rhs.z
    }
}

impl fmt::Display for Coordinate {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "(x:{}, y:{}, z:{})", self.x, self.y, self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_norm() {
        let a = Coordinate {
            x: 1.,
            y: 2.,
            z: 3.,
        };
        assert_eq!(a.norm(), 14.0_f64.sqrt());
    }

    #[test]
    fn test_inner_product() {
        let a = Coordinate {
            x: 1.,
            y: 2.,
            z: 3.,
        };
        let b = Coordinate {
            x: 4.,
            y: 5.,
            z: 6.,
        };
        assert_eq!(a.inner_product(b), 32.0);
    }

    #[test]
    fn test_cross_product() {
        let a = Coordinate {
            x: 1.,
            y: 2.,
            z: 3.,
        };
        let b = Coordinate {
            x: 4.,
            y: 5.,
            z: 6.,
        };
        assert_eq!(
            a.cross_product(b),
            Coordinate {
                x: -3.,
                y: 6.,
                z: -3.,
            }
        );
    }
}

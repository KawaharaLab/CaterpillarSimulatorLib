use std::fmt;
use std::f64;
use coordinate;

#[derive(Copy, Clone)]
pub struct TorsionSpring {
    spring_constant: f64,
    standard_vector: coordinate::Coordinate, // torsion is calculated within an orthogonal plane to the standard_vector
}

impl TorsionSpring {
    pub fn new(spring_constant: f64, standard: coordinate::Coordinate) -> Self {
        let epsilon = 1.0e-10;
        if standard.norm() <= 1. - epsilon || standard.norm() >= 1. + epsilon {
            panic!("norm of standard_vector should be 1.0, {}", standard)
        }
        TorsionSpring {
            spring_constant: spring_constant,
            standard_vector: standard,
        }
    }

    pub fn force(
        &self,
        base: coordinate::Coordinate,
        center: coordinate::Coordinate,
        tip: coordinate::Coordinate,
        target_angle: f64,
    ) -> coordinate::Coordinate {
        // calculate torsion force applied on tip, so that Arg(base-center, center-tip) anti-clock-wise to standard_vector becomes target_angular
        // base and tip are not symmetric, i.e. if you swap base and tip you should modify target_angular to (2*PI - original_target_angle)
        // range of target_angle is [0, 2*PI], if it exceeds target_angle % 2*PI will be used
        let vec_bc = center - base;
        let vec_ct = tip - center;
        self.normal_vector(vec_ct) * -self.spring_constant
            * (self.angle(vec_bc, vec_ct) - target_angle)
    }

    fn normal_vector(&self, v: coordinate::Coordinate) -> coordinate::Coordinate {
        // anti-clock-wise orthogonal vector to v, whose norm is 1
        self.standard_vector.cross_product(v) / v.norm()
    }

    fn angle(&self, v1: coordinate::Coordinate, v2: coordinate::Coordinate) -> f64 {
        // Arg(v1, v2) anti-clock-wise to the standard_vector
        if self.sin(v1, v2) >= 0.0 {
            self.cos(v1, v2).acos()
        } else {
            2. * f64::consts::PI - self.cos(v1, v2).acos()
        }
    }

    fn cos(&self, v1: coordinate::Coordinate, v2: coordinate::Coordinate) -> f64 {
        let v1_ = self.project(v1);
        let v2_ = self.project(v2);
        v1_.inner_product(v2_) / (v1_.norm() * v2_.norm())
    }

    fn sin(&self, v1: coordinate::Coordinate, v2: coordinate::Coordinate) -> f64 {
        let v1_ = self.project(v1);
        let v2_ = self.project(v2);
        let cross = v1_.cross_product(v2_);
        cross.norm() / (v1_.norm() * v2_.norm())
            * cross.inner_product(self.standard_vector).signum()
    }

    fn project(&self, v: coordinate::Coordinate) -> coordinate::Coordinate {
        v - self.standard_vector * self.standard_vector.inner_product(v)
    }
}

impl fmt::Display for TorsionSpring {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "TorsionSpring   spring constant: {}",
            self.spring_constant
        )
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    #[should_panic]
    fn test_panick_on_invalid_standard_vector() {
        TorsionSpring::new(
            0.,
            coordinate::Coordinate {
                x: 1.,
                y: 1.,
                z: 1.,
            },
        );
    }

    #[test]
    fn test_angle() {
        let torsion_spring = TorsionSpring::new(
            0.,
            coordinate::Coordinate {
                x: 0.,
                y: 0.,
                z: 1.,
            },
        );
        let v1 = coordinate::Coordinate {
            x: 1.,
            y: 1.,
            z: 1.,
        };
        let v2 = coordinate::Coordinate {
            x: 1.,
            y: -1.,
            z: -2.,
        };
        assert_eq!(torsion_spring.angle(v1, v2), 1.5 * f64::consts::PI)
    }
}

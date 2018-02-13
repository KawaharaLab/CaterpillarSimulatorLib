use std::fmt;
use std::f64;
use coordinate;

const EPSILON: f64 = 1.0e-5;

#[derive(Copy, Clone)]
pub struct TorsionSpring {
    spring_constant_k0: f64,
    spring_constant_k1: f64,
    standard_vector: coordinate::Coordinate, // torsion is calculated within an orthogonal plane to the standard_vector
}

impl TorsionSpring {
    pub fn new(
        spring_constant_k0: f64,
        spring_constant_k1: f64,
        standard: coordinate::Coordinate,
    ) -> Self {
        let epsilon = 1.0e-10;
        if standard.norm() <= 1. - epsilon || standard.norm() >= 1. + epsilon {
            panic!("norm of standard_vector should be 1.0, {}", standard)
        }
        TorsionSpring {
            spring_constant_k0: spring_constant_k0,
            spring_constant_k1: spring_constant_k1,
            standard_vector: standard,
        }
    }

    pub fn force(
        &self,
        base: coordinate::Coordinate,
        center: coordinate::Coordinate,
        tip: coordinate::Coordinate,
        target_angle: f64,
    ) -> (coordinate::Coordinate, coordinate::Coordinate) {
        // calculate torsion force applied on tip and base, so that Arg(base-center, center-tip) anti-clock-wise to standard_vector becomes target_angular.
        // force applied on tip and base is or symmetrical.
        // base and tip are not symmetric, i.e. if you swap base and tip you should modify target_angular to (2*PI - original_target_angle).
        // range of target_angle is [0, 2*PI], if it exceeds target_angle % 2*PI will be used.
        let vec_bc = center - base;
        let vec_ct = tip - center;
        let angle_diff = self.angle(vec_bc, vec_ct) - target_angle;
        if angle_diff.abs() < EPSILON {
            // take into account numeric error
            (
                coordinate::Coordinate::zero(),
                coordinate::Coordinate::zero(),
            )
        } else {
            (
                self.normal_vector(vec_ct) * -self.calculate_spring_constant(angle_diff)
                    * angle_diff,
                self.normal_vector(vec_bc) * -self.calculate_spring_constant(angle_diff)
                    * angle_diff,
            )
        }
    }

    pub fn force_on_discrepancy(
        &self,
        base: coordinate::Coordinate,
        center: coordinate::Coordinate,
        tip: coordinate::Coordinate,
        discrepancy_angle_angle: f64,
    ) -> (coordinate::Coordinate, coordinate::Coordinate) {
        // calculate torsion force applied on tip and base, so that discrepancy_angle_angle becomse zero.
        // discrepancy_angle is angle from actual position to target position.
        // force applied on tip and base is or symmetrical.
        // base and tip are not symmetric, i.e. if you swap base and tip you should modify target_angular to (2*PI - original_target_angle).
        // range of target_angle is [0, 2*PI], if it exceeds target_angle % 2*PI will be used.
        let vec_bc = center - base;
        let vec_ct = tip - center;
        if discrepancy_angle_angle < EPSILON {
            // take into account numeric error
            (
                coordinate::Coordinate::zero(),
                coordinate::Coordinate::zero(),
            )
        } else {
            (
                self.normal_vector(vec_ct) * self.calculate_spring_constant(discrepancy_angle_angle)
                    * discrepancy_angle_angle, // no minus since discrepancy_angle is from actual to target position
                self.normal_vector(vec_bc) * self.calculate_spring_constant(discrepancy_angle_angle)
                    * discrepancy_angle_angle,
            )
        }
    }

    pub fn current_angle(
        &self,
        base: coordinate::Coordinate,
        center: coordinate::Coordinate,
        tip: coordinate::Coordinate,
    ) -> f64 {
        // 1.0 if current angle -> target angle is anti-clock-wise
        // -1.0 if current angle -> target angle is anti-clock-wise
        let vec_bc = center - base;
        let vec_ct = tip - center;
        self.angle(vec_bc, vec_ct)
    }

    fn calculate_spring_constant(&self, target_angle: f64) -> f64 {
        self.spring_constant_k0 + self.spring_constant_k1 * target_angle.abs()
    }

    fn normal_vector(&self, v: coordinate::Coordinate) -> coordinate::Coordinate {
        // anti-clock-wise orthogonal vector to v, whose norm is 1
        self.standard_vector.cross_product(self.project(v)) / self.project(v).norm()
    }

    fn angle(&self, v1: coordinate::Coordinate, v2: coordinate::Coordinate) -> f64 {
        // Arg(v1, v2) anti-clock-wise to the standard_vector
        if self.sin(v1, v2) >= 0.0 {
            self.cos(v1, v2).acos()
        } else {
            -self.cos(v1, v2).acos()
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
            "TorsionSpring   spring constant k0: {}   k1: {}",
            self.spring_constant_k0, self.spring_constant_k1
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
        let torsion_spring1 = TorsionSpring::new(
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
        let angle1 = torsion_spring1.angle(v1, v2);
        let expected1 = -0.5 * f64::consts::PI;
        assert!(expected1 - EPSILON < angle1 && angle1 < expected1 + EPSILON);

        let torsion_spring2 = TorsionSpring::new(
            0.,
            coordinate::Coordinate {
                x: 0.,
                y: 1.,
                z: 0.,
            },
        );
        let v3 = coordinate::Coordinate {
            x: 1.,
            y: 1.,
            z: 1.,
        };
        let v4 = coordinate::Coordinate {
            x: 2.,
            y: -1.,
            z: 0.,
        };
        let angle2 = torsion_spring2.angle(v3, v4);
        let expected2 = 0.25 * f64::consts::PI;
        assert!(expected2 - EPSILON < angle2 && angle2 < expected2 + EPSILON);
    }

    #[test]
    fn test_normal_vector() {
        let torsion_spring = TorsionSpring::new(
            0.,
            coordinate::Coordinate {
                x: 0.,
                y: 1.,
                z: 0.,
            },
        );
        let v = coordinate::Coordinate {
            x: 1.,
            y: 2.,
            z: 1.,
        };
        let result = torsion_spring.normal_vector(v);
        let expected = coordinate::Coordinate {
            x: 2.0_f64.powf(-0.5),
            y: 0.,
            z: -2.0_f64.powf(-0.5),
        };
        assert!(
            expected.x - EPSILON < result.x && result.x < expected.x + EPSILON,
            format!("result x :{}   expected: {}", result.x, expected.x)
        );
        assert!(
            expected.y - EPSILON < result.y && result.y < expected.y + EPSILON,
            format!("result y :{}   expected: {}", result.y, expected.y)
        );
        assert!(
            expected.z - EPSILON < result.z && result.z < expected.z + EPSILON,
            format!("result z :{}   expected: {}", result.z, expected.z)
        );
    }
}

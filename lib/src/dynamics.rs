use std::cell::Ref;
use coordinate::Coordinate;
use somite::Somite;
use phase_oscillator::PhaseOscillator;

/// Dynamics defines mechanical dynamics of a system
/// 
/// # Example
/// 
/// ```
/// let dy = Dynamics{
///     gripper_k: 1.,
///     gripper_c: 2.,
///     dynamic_friction_coeff: 3.,
///     static_friction_coeff: 4.,
///     viscosity_friction_coeff: 5.,
///     grip_phase_threshold: 6.,
/// };
/// ```
/// 
#[derive(Default)]
pub struct Dynamics {
    pub gripper_k: f64,
    pub gripper_c: f64,
    pub dynamic_friction_coeff: f64,
    pub static_friction_coeff: f64,
    pub viscosity_friction_coeff: f64,
    pub grip_phase_threshold: f64,
}

impl Dynamics {
    /// Calculate force caused by a gripper.
    /// Gripping force is modeled by strong springs.
    /// The gripping force only acts when gripping.
    pub fn calculate_somite_shear_force(&self, somite: &Somite, applied_force: &Coordinate, has_leg: bool) -> Coordinate {
        if has_leg {
            if let Some(grip_point) = somite.get_gripping_point() {
                // gripping
                self.gripping_force(&grip_point, &somite.get_position(), &somite.get_verocity())
            } else {
                if somite.is_on_ground() {
                    // not gripping but on the ground
                    let f_x = self.shear_friction(somite.get_verocity(), applied_force);
                    Coordinate::new(f_x, 0., 0.)
                } else {
                    // in the air
                    Coordinate::zero()
                }
            }
        } else {
            if somite.is_on_ground() {
                // no gripper but on the ground
                Coordinate::new(self.shear_friction(somite.get_verocity(), applied_force), 0., 0.)
            } else {
                // in the air
                Coordinate::zero()
            }
        }
    }

    fn gripping_force(&self, resting_point: &Coordinate, current_point: &Coordinate, verocity: &Coordinate) -> Coordinate{
        Coordinate::new(
            -self.gripper_k * (current_point.x - resting_point.x) - self.gripper_c * verocity.x,
            0.,
            -self.gripper_k * (current_point.z - resting_point.z) - self.gripper_c * verocity.z,
        )
    }

    fn shear_friction(&self, verocity: Coordinate, applied_force: &Coordinate) -> f64 {
        let normal_force = applied_force.z.min(0.).abs();
        if verocity.x.abs() > 0. {
            -verocity.x.signum() * self.dynamic_friction_coeff * normal_force
                + self.viscosity_friction_coeff * -verocity.x
        } else {
            let max_static_friction = self.static_friction_coeff * normal_force;
            if applied_force.x.abs() > max_static_friction {
                max_static_friction * -applied_force.x.signum()
            } else {
                -applied_force.x
            }
        }
    }

    pub fn should_grip(&self, somite: &Somite, oscillator: Ref<PhaseOscillator>) -> bool {
        if oscillator.get_phase().sin() < self.grip_phase_threshold && somite.is_on_ground()
            && !somite.is_gripping()
        {
            true
        } else {
            false
        }
    }

    pub fn should_release(&self, somite: &Somite, oscillator: Ref<PhaseOscillator>) -> bool {
        if oscillator.get_phase().sin() >= self.grip_phase_threshold && somite.is_gripping() {
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod test {
    use std::f64;
    use std::cell::RefCell;
    use super::*;
    use coordinate::Coordinate;
    use somite::Somite;
    use phase_oscillator::PhaseOscillator;

    #[test]
    fn test_calculate_shear_force_while_gripping() {
        let d = Dynamics {
            gripper_k: 10.,
            gripper_c: 20.,
            ..Default::default()
        };
        let s = Somite::new(
            1.,
            1.,
            Coordinate::new(0., 0., 1.),
            Coordinate::new(-2., 0., 0.),
        );
        s.grip();
        s.set_position(Coordinate::new(1., 0., 1.));
        let force_applied = Coordinate::new(5., 0., -6.);
        let shear_force = d.calculate_somite_shear_force(&s, &force_applied, true);
        let expected = Coordinate::new(-d.gripper_c * -2. + -d.gripper_k * 1., 0., 0.);
        assert_eq!(
            shear_force, expected,
            "while gripping, expected {}, got {}",
            expected, shear_force
        );
    }

    #[test]
    fn test_calculate_shear_force_in_air() {
        let d = Dynamics {
            ..Default::default()
        };
        let s = Somite::new(
            1.,
            1.,
            Coordinate::new(0., 0., 1.1),
            Coordinate::new(-2., 0., 0.),
        );
        let force_applied = Coordinate::new(5., 0., -6.);
        let shear_force = d.calculate_somite_shear_force(&s, &force_applied, true);
        let expected = Coordinate::new(0., 0., 0.);
        assert_eq!(
            shear_force, expected,
            "while in the air, expected {}, got {}",
            expected, shear_force
        );
    }

    #[test]
    fn test_calculate_shear_force_while_released_and_moving() {
        let d = Dynamics {
            dynamic_friction_coeff: 3.,
            viscosity_friction_coeff: 4.,
            ..Default::default()
        };
        let s = Somite::new(
            1.,
            1.,
            Coordinate::new(0., 0., 1.),
            Coordinate::new(-2., 0., 0.),
        );
        let force_applied = Coordinate::new(5., 0., -6.);
        let shear_force = d.calculate_somite_shear_force(&s, &force_applied, true);
        let expected = Coordinate::new(
            -d.dynamic_friction_coeff * (-6.0_f64).abs() * (-1.)
                - d.viscosity_friction_coeff * (-2.),
            0.,
            0.,
        );
        assert_eq!(
            shear_force, expected,
            "while released and moving, expected {}, got {}",
            expected, shear_force
        );
    }

    #[test]
    fn test_calculate_shear_force_while_released_and_still() {
        let d = Dynamics {
            static_friction_coeff: 3.,
            ..Default::default()
        };
        let s = Somite::new(
            1.,
            1.,
            Coordinate::new(0., 0., 1.),
            Coordinate::new(0., 0., 0.),
        );
        let force_applied = Coordinate::new(5., 0., -6.);
        let shear_force = d.calculate_somite_shear_force(&s, &force_applied, true);
        let expected = Coordinate::new(-5., 0., 0.);
        assert_eq!(
            shear_force, expected,
            "while released and moving, expected {}, got {}",
            expected, shear_force
        );
    }

    #[test]
    fn test_calculate_shear_force_while_released_and_still_beyound_max_static_friction() {
        let d = Dynamics {
            static_friction_coeff: 3.,
            ..Default::default()
        };
        let s = Somite::new(
            1.,
            1.,
            Coordinate::new(0., 0., 1.),
            Coordinate::new(0., 0., 0.),
        );
        let force_applied = Coordinate::new(20., 0., -6.);
        let shear_force = d.calculate_somite_shear_force(&s, &force_applied, true);
        let expected = Coordinate::new(
            -d.static_friction_coeff * (-6.0_f64).abs() * (20.0_f64).signum(),
            0.,
            0.,
        );
        assert_eq!(
            shear_force, expected,
            "while released, moving, and applied force is larger than max static friction, expected {}, got {}",
            expected, shear_force
        );
    }

    #[test]
    fn test_should_grip() {
        let d = Dynamics {
            grip_phase_threshold: -0.5,
            ..Default::default()
        };
        let s = Somite::new(1., 1., Coordinate::new(0., 0., 1.), Coordinate::zero());
        let o = RefCell::<PhaseOscillator>::new(PhaseOscillator::new());

        s.set_position(Coordinate::new(0., 0., 1.1));
        o.borrow_mut().set_phase(f64::consts::PI);
        assert!(
            !d.should_grip(&s, o.borrow()),
            "in the air & out of grip range"
        );

        o.borrow_mut().set_phase(3. / 2. * f64::consts::PI);
        assert!(
            !d.should_grip(&s, o.borrow()),
            "in the air & in the grip range"
        );

        s.set_position(Coordinate::new(0., 0., 1.));
        o.borrow_mut().set_phase(f64::consts::PI);
        assert!(
            !d.should_grip(&s, o.borrow()),
            "on the ground & out of grip range"
        );

        o.borrow_mut().set_phase(3. / 2. * f64::consts::PI);
        s.grip();
        assert!(
            !d.should_grip(&s, o.borrow()),
            "on the ground & in the grip range & gripping"
        );

        s.release();
        assert!(
            d.should_grip(&s, o.borrow()),
            "on the ground & in the grip range & not gripping"
        );
    }

    #[test]
    fn test_should_release() {
        let d = Dynamics {
            grip_phase_threshold: -0.5,
            ..Default::default()
        };
        let s = Somite::new(1., 1., Coordinate::new(0., 0., 1.), Coordinate::zero());
        let o = RefCell::<PhaseOscillator>::new(PhaseOscillator::new());

        s.set_position(Coordinate::new(0., 0., 1.1));
        o.borrow_mut().set_phase(f64::consts::PI);
        assert!(
            !d.should_release(&s, o.borrow()),
            "in the air & out of grip range"
        );

        o.borrow_mut().set_phase(3. / 2. * f64::consts::PI);
        assert!(
            !d.should_release(&s, o.borrow()),
            "in the air & in the grip range"
        );

        s.set_position(Coordinate::new(0., 0., 1.));
        o.borrow_mut().set_phase(f64::consts::PI);
        assert!(
            !d.should_release(&s, o.borrow()),
            "on the ground & out of grip range & not gripping"
        );

        s.grip();
        assert!(
            d.should_release(&s, o.borrow()),
            "on the ground & out of grip range & gripping"
        );

        o.borrow_mut().set_phase(3. / 2. * f64::consts::PI);
        assert!(
            !d.should_release(&s, o.borrow()),
            "on the ground & in the grip range & gripping"
        );

        s.release();
        assert!(
            !d.should_release(&s, o.borrow()),
            "on the ground & in the grip range & not gripping"
        );
    }
}

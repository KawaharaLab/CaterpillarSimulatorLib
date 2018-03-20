use std::cell::Ref;
use coordinate::Coordinate;
use somite::Somite;
use phase_oscillator::PhaseOscillator;

#[derive(Default)]
pub struct Dynamics {
    pub shear_force_k: f64,
    pub shear_force_c: f64,
    pub dynamic_friction_coeff: f64,
    pub viscosity_friction_coeff: f64,
    pub grip_phase_threshold: f64,
}

impl Dynamics {
    pub fn calculate_somite_shear_force(
        &self,
        somite: &Somite,
        normal_force: f64,
        has_leg: bool,
    ) -> Coordinate {
        let fx = if has_leg {
            if let Some(grip_point) = somite.get_gripping_point() {
                // gripping
                self.grip_shear_force(
                    grip_point.x,
                    somite.get_position().x,
                    somite.get_verocity().x,
                )
            } else {
                if somite.is_on_ground() {
                    // not gripping but on the ground
                    self.shear_friction(somite.get_verocity().x, normal_force)
                } else {
                    // in the air
                    0.
                }
            }
        } else {
            if somite.is_on_ground() {
                // no gripper but on the ground
                self.shear_friction(somite.get_verocity().x, normal_force)
            } else {
                // in the air
                0.
            }
        };
        Coordinate::new(fx, 0., 0.)
    }

    fn grip_shear_force(&self, resting_point: f64, current_point: f64, verocity: f64) -> f64 {
        -self.shear_force_k * (current_point - resting_point) - self.shear_force_c * verocity
    }

    fn shear_friction(&self, verocity: f64, normal_force: f64) -> f64 {
        debug_assert!(normal_force >= 0.);
        -verocity.signum() * self.dynamic_friction_coeff * normal_force
            - self.viscosity_friction_coeff * verocity
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
    fn test_calculate_somite_shear_force() {
        let d = Dynamics {
            shear_force_k: 10.,
            shear_force_c: 20.,
            dynamic_friction_coeff: 3.,
            viscosity_friction_coeff: 4.,
            ..Default::default()
        };
        let s = Somite::new(
            1.,
            1.,
            Coordinate::new(0., 0., 0.),
            Coordinate::new(-1., 0., 0.),
        );
        s.grip();
        s.set_position(Coordinate::new(1., 0., 0.));
        let shear_force = d.calculate_somite_shear_force(&s, 3., true);
        let expected = Coordinate::new(10., 0., 0.);
        assert_eq!(
            shear_force, expected,
            "expected {}, got {}",
            expected, shear_force
        );

        s.release();
        let shear_force = d.calculate_somite_shear_force(&s, 3., true);
        let expected = Coordinate::new(13., 0., 0.);
        assert_eq!(
            shear_force, expected,
            "expected {}, got {}",
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
use std::cell::Ref;
use coordinate::Coordinate;
use somite::Somite;
use phase_oscillator::PhaseOscillator;
use path_heights::PathHeights;

/// Dynamics defines mechanical dynamics of a system
/// 
/// # Example
/// 
/// ```
/// let dy = Dynamics{
///     shear_force_k: 1.,
///     shear_force_c: 2.,
///     dynamic_friction_coeff: 3.,
///     static_friction_coeff: 4.,
///     viscosity_friction_coeff: 5.,
///     grip_phase_threshold: 6.,
/// };
/// ```
/// 
#[derive(Default)]
pub struct Dynamics {
    pub shear_force_k: f64,
    pub shear_force_c: f64,
    pub dynamic_friction_coeff: f64,
    pub static_friction_coeff: f64,
    pub viscosity_friction_coeff: f64,
    pub grip_phase_threshold: f64,
}

const EPSILON: f64 = 10e-5;
const STUCKED_EPSILON: f64 = 10e-3;

impl Dynamics {
    /// Calculate shear force caused by friction between a somite and the substrate.
    pub fn calculate_friction(&self, somite: &Somite, applied_force: &Coordinate) -> Coordinate {
        Coordinate::new(self.shear_friction(somite.get_verocity(), applied_force), 0., 0.)
    }

    /// Calculate force caused by a gripper.
    /// Gripping force is modeled by strong springs.
    /// The gripping force only acts when gripping.
    pub fn calculate_gripping_force(&self, somite: &Somite, applied_force: &Coordinate) -> Coordinate {
        Coordinate::new(
            self.grip_shear_force(somite.get_gripping_point().unwrap().x, somite.get_position().x, somite.get_verocity().x),
            0.,
            - applied_force.z, // cancel force along z axis if gripping
        )
    }

    fn grip_shear_force(&self, resting_point: f64, current_point: f64, verocity: f64) -> f64 {
        -self.shear_force_k * (current_point - resting_point) - self.shear_force_c * verocity
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

    pub fn is_blocked_by_obstacle(&self, somite: &Somite, path_height: &PathHeights) -> bool {
        somite.get_position().z < somite.radius + path_height.get_height(somite.get_position().x) - EPSILON
    }

    pub fn should_grip(&self, somite: &Somite, oscillator: Ref<PhaseOscillator>, path_heights: &PathHeights) -> bool {
        if oscillator.get_phase().sin() < self.grip_phase_threshold && path_heights.is_on_ground(somite, self.is_blocked_by_obstacle(somite, path_heights))
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
    fn test_is_blocked_by_obstacle() {
        let d = Dynamics {..Default::default()};
        let s = Somite::new(
            1.,
            1.,
            Coordinate::new(0.4, 0., 1.),
            Coordinate::zero(),
        );
        let mut path_heights = PathHeights::new();
        path_heights.set(0.5, 0.7).unwrap();
        
        // not blocked
        assert!(!d.is_blocked_by_obstacle(&s, &path_heights));

        // blocked
        s.set_position(Coordinate::new(0.51, 0., 1.));
        assert!(d.is_blocked_by_obstacle(&s, &path_heights));

        // not blocked
        s.set_position(Coordinate::new(0.51, 0., 1.7));
        assert!(!d.is_blocked_by_obstacle(&s, &path_heights));
    }

    #[test]
    fn test_calculate_gripping_force() {
        let d = Dynamics {
            shear_force_k: 10.,
            shear_force_c: 20.,
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
        let gripping_force = d.calculate_gripping_force(&s, &force_applied);
        let expected = Coordinate::new(-d.shear_force_c * -2. + -d.shear_force_k * 1., 0., 6.);
        assert_eq!(
            gripping_force, expected,
            "while gripping, expected {}, got {}",
            expected, gripping_force
        );
    }

    #[test]
    fn test_calculate_friction_dynamic() {
        let d = Dynamics {
            dynamic_friction_coeff: 3.,
            viscosity_friction_coeff: 4.,
            ..Default::default()
        };
        let s = Somite::new(1., 1., Coordinate::new(0., 0., 1.), Coordinate::new(-2., 0., 0.));
        let force_applied = Coordinate::new(5., 0., -6.);
        let friction = d.calculate_friction(&s, &force_applied);
        let expected = Coordinate::new(
            -d.dynamic_friction_coeff * (-6.0_f64).abs() * (-1.) - d.viscosity_friction_coeff * (-2.),
            0.,
            0.,
        );
        assert_eq!(
            friction, expected,
            "while released and moving, expected {}, got {}",
            expected, friction
        );
    }

    #[test]
    fn test_calculate_friction_static() {
        let d = Dynamics {
            static_friction_coeff: 3.,
            ..Default::default()
        };
        let s = Somite::new(1., 1., Coordinate::new(0., 0., 1.), Coordinate::new(0., 0., 0.));
        let force_applied = Coordinate::new(5., 0., -6.);
        let friction = d.calculate_friction(&s, &force_applied);
        let expected = Coordinate::new(-5., 0., 0.);
        assert_eq!(
            friction, expected,
            "while released and moving, expected {}, got {}",
            expected, friction
        );
    }

    #[test]
    fn test_calculate_friction_maximum_static() {
        let d = Dynamics {
            static_friction_coeff: 3.,
            ..Default::default()
        };
        let s = Somite::new( 1., 1., Coordinate::new(0., 0., 1.), Coordinate::new(0., 0., 0.));
        let force_applied = Coordinate::new(20., 0., -6.);
        let friction = d.calculate_friction(&s, &force_applied);
        let expected = Coordinate::new( -d.static_friction_coeff * (-6.0_f64).abs() * (20.0_f64).signum(), 0., 0.);
        assert_eq!(
            friction, expected,
            "while released, moving, and applied force is larger than max static friction, expected {}, got {}",
            expected, friction
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
            !d.should_grip(&s, o.borrow(), &PathHeights::new()),
            "in the air & out of grip range"
        ); 

        o.borrow_mut().set_phase(3. / 2. * f64::consts::PI);
        assert!(
            !d.should_grip(&s, o.borrow(), &PathHeights::new()),
            "in the air & in the grip range"
        );

        s.set_position(Coordinate::new(0., 0., 1.));
        o.borrow_mut().set_phase(f64::consts::PI);
        assert!(
            !d.should_grip(&s, o.borrow(), &PathHeights::new()),
            "on the ground & out of grip range"
        );

        o.borrow_mut().set_phase(3. / 2. * f64::consts::PI);
        s.grip();
        assert!(
            !d.should_grip(&s, o.borrow(), &PathHeights::new()),
            "on the ground & in the grip range & gripping"
        );

        s.release();
        assert!(
            d.should_grip(&s, o.borrow(), &PathHeights::new()),
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

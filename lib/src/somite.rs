use std::fmt;
use std::cell;
use coordinate;

const EPSILON: f64 = 1.0e-5;

pub struct Somite {
    pub position: cell::Cell<coordinate::Coordinate>,
    pub verocity: cell::Cell<coordinate::Coordinate>,
    pub force: cell::Cell<coordinate::Coordinate>,
    pub radius: f64,
    pub mass: f64,
    gripping_flag: cell::Cell<bool>,
    gripping_point: cell::RefCell<coordinate::Coordinate>,
}

impl fmt::Display for Somite {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "Somite position: {}, verocity: {}, force: {}",
            self.position.get(),
            self.verocity.get(),
            self.force.get()
        )
    }
}

impl Somite {
    pub fn new(
        radius: f64,
        mass: f64,
        position: coordinate::Coordinate,
        verocity: coordinate::Coordinate,
    ) -> Self {
        Somite {
            position: cell::Cell::new(position),
            verocity: cell::Cell::new(verocity),
            force: cell::Cell::new(coordinate::Coordinate {
                x: 0.,
                y: 0.,
                z: 0.,
            }),
            radius: radius,
            mass: mass,
            gripping_flag: cell::Cell::new(false),
            gripping_point: cell::RefCell::<coordinate::Coordinate>::new(
                coordinate::Coordinate::zero(),
            ),
        }
    }

    pub fn new_still_somite(radius: f64, mass: f64, position: coordinate::Coordinate) -> Self {
        Self::new(
            radius,
            mass,
            position,
            coordinate::Coordinate {
                x: 0.,
                y: 0.,
                z: 0.,
            },
        )
    }

    pub fn set_position(&self, position: coordinate::Coordinate) {
        self.position.set(position);
    }

    pub fn get_position(&self) -> coordinate::Coordinate {
        self.position.get()
    }

    pub fn set_verocity(&self, verocity: coordinate::Coordinate) {
        self.verocity.set(verocity);
    }

    pub fn get_verocity(&self) -> coordinate::Coordinate {
        self.verocity.get()
    }

    pub fn set_force(&self, acceleration: coordinate::Coordinate) {
        self.force.set(acceleration);
    }

    pub fn get_force(&self) -> coordinate::Coordinate {
        self.force.get()
    }

    pub fn get_verocity_direction_x(&self) -> f64 {
        // if v_x > 0, return 1.0
        // if v_x < 0, return -1.0
        unsafe { *self.verocity.as_ptr() }.x.signum()
    }

    pub fn get_verocity_direction_y(&self) -> f64 {
        // if v_y > 0, return 1.0
        // if v_y < 0, return -1.0
        unsafe { *self.verocity.as_ptr() }.y.signum()
    }

    pub fn is_on_ground(&self) -> bool {
        self.position.get().z <= self.radius
    }

    pub fn is_moving_x(&self) -> bool {
        unsafe { *self.verocity.as_ptr() }.x.abs() > EPSILON
    }

    pub fn is_moving_y(&self) -> bool {
        unsafe { *self.verocity.as_ptr() }.y.abs() > EPSILON
    }

    pub fn is_gripping(&self) -> bool {
        self.gripping_flag.get()
    }

    pub fn grip(&self) {
        // update gripping point only if gripping flag is false
        if !self.gripping_flag.get() {
            self.gripping_flag.set(true);
            let p = self.position.get();
            self.gripping_point.borrow_mut().x = p.x;
            self.gripping_point.borrow_mut().y = p.y;
            self.gripping_point.borrow_mut().z = p.z;
        }
    }

    pub fn release(&self) {
        self.gripping_flag.set(false);
    }

    pub fn get_gripping_point(&self) -> Option<cell::Ref<coordinate::Coordinate>> {
        if self.gripping_flag.get() {
            Some(self.gripping_point.borrow())
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grip() {
        let s = Somite::new_still_somite(1., 2., coordinate::Coordinate::new(0., 0., 4.));
        s.grip();
        assert_eq!(
            *s.get_gripping_point().unwrap(),
            coordinate::Coordinate::new(0., 0., 4.)
        );

        // don't update gripping point if already gripping
        s.set_position(coordinate::Coordinate::new(1., 0., 4.));
        s.grip();
        assert_eq!(
            *s.get_gripping_point().unwrap(),
            coordinate::Coordinate::new(0., 0., 4.)
        );
    }

}

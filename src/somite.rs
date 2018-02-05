use std::fmt;
use std::cell;
use coordinate;

pub struct Somite {
    pub position: cell::Cell<coordinate::Coordinate>,
    pub verocity: cell::Cell<coordinate::Coordinate>,
    pub force: cell::Cell<coordinate::Coordinate>,
    pub radius: f64,
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
        }
    }

    pub fn new_still_somite(radius: f64, position: coordinate::Coordinate) -> Self {
        Self::new(
            radius,
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

    pub fn calculate_friction(&self, friction_coeff: f64) -> coordinate::Coordinate {
        if self.position.get().z <= self.radius {
            self.verocity.get() * -friction_coeff
        } else {
            coordinate::Coordinate::zero()
        }
    }
}

use coordinate;

const EPSILON: f64 = 1.0e-5;

pub struct Spring {
    spring_constant: f64,
    natural_length: f64,
}

impl Spring {
    pub fn new(spring_constant: f64, natural_length: f64) -> Self {
        Spring {
            spring_constant: spring_constant,
            natural_length: natural_length,
        }
    }

    pub fn force(
        &self,
        base: coordinate::Coordinate,
        tip: coordinate::Coordinate,
    ) -> coordinate::Coordinate {
        // Calculate tension. Positive direction is base -> tip
        let length = (tip - base).norm();
        let length_diff = length - self.natural_length;
        if length_diff.abs() < EPSILON {
            // take into account numeric error
            coordinate::Coordinate::zero()
        } else {
            let n = (tip - base) / length;
            n * -self.spring_constant * length_diff
        }
    }
}

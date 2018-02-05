use coordinate;

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
        let n = (tip - base) / length;
        n * -self.spring_constant * (length - self.natural_length)
    }
}

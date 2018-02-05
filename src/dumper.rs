use coordinate;

pub struct Dumper {
    dumper_coefficient: f64,
}

impl Dumper {
    pub fn new(dumper_coefficient: f64) -> Self {
        Dumper {
            dumper_coefficient: dumper_coefficient,
        }
    }

    pub fn force(
        &self,
        base_verocity: coordinate::Coordinate,
        tip_verocity: coordinate::Coordinate,
    ) -> coordinate::Coordinate {
        // Calculate dumping force. Positive direction is base -> tip
        (tip_verocity - base_verocity) * -self.dumper_coefficient
    }
}

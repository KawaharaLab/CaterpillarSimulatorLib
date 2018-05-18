use somite::Somite;

/// PathHeights holds stepwise path heights.
/// Each step (section) is represented by its start point and height.
pub struct PathHeights {
    start_points: Vec<f64>,
    heights: Vec<f64>,
}

impl PathHeights {
    /// Create new path.
    /// Default to plain path, i.e., {0.: 0.}
    pub fn new() -> Self {
        PathHeights{start_points: vec![0.], heights: vec![0.]}
    }

    /// set inserts new section beginning and its height
    pub fn set(&mut self, start_point: f64, height: f64) -> Result<(), String> {
        if start_point < 0. {
            Err("start_point cannot be negative".to_owned())
        } else {
            self.start_points.push(start_point);
            self.heights.push(height);
            Ok(())
        }
    }

    /// is_on_ground returns true if a given object is on the ground, and false otherwise 
    pub fn is_on_ground(&self, s: &Somite) -> bool {
        let x = &s.get_position().x;
        if x < self.start_points.first().unwrap() {
            return s.is_on_ground(*self.heights.first().unwrap());
        }
        for (i, start_point) in self.start_points.iter().enumerate() {
            if start_point > x  {
                return s.is_on_ground(self.heights[i-1]); // the first start_point is 0, thus i>1
            }
        }
        s.is_on_ground(*self.heights.last().unwrap()) // at least, 0 is set
    }

    pub fn get_height(&self, x: f64) -> f64 {
        if x < *self.start_points.first().unwrap() {
            return *self.heights.first().unwrap();
        }
        for (i, start_point) in self.start_points.iter().enumerate() {
            if *start_point > x {
                return self.heights[i-1];
            }
        }
        *self.heights.last().unwrap()
    }
}   
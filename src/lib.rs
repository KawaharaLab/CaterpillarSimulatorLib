#[macro_use]
extern crate cpython;

#[macro_use]
extern crate serde_derive;

use std::cell;
use cpython::{PyObject, PyResult, PyString, PyTuple, Python, ToPyObject};

mod somite;
mod torsion_spring;
mod spring;
mod dumper;
mod caterpillar_config;
mod coordinate;
mod simulation_export;

use somite::Somite;
use torsion_spring::TSP;

const CONFIG: caterpillar_config::Config = caterpillar_config::Config {
    somite_mass: 0.3,
    somite_radius: 0.35,
    normal_angular_velocity: 0.35,
    rts_max_natural_length: std::f64::consts::PI,
    rts_k: 100.0,
    rts_c: 1.0,
    rts_amp: 0.3,
    sp_natural_length: 0.7,
    sp_k: 100.0,
    dp_c: 1.0,
    friction_coeff: 1.0,
    time_delta: 0.01,
};

py_module_initializer!(caterpillar, initcaterpillar, PyInit_caterpillar, |py, m| {
    try!(m.add(
        py,
        "__doc__",
        "This is Rust implementation of caterpillar simulater."
    ));
    try!(m.add_class::<Caterpillar>(py));
    Ok(())
});

py_class!(class Caterpillar |py| {
    data somites: Vec<Somite>;
    data simulation_protocol: simulation_export::SimulationProc;
    data frame_count: cell::Cell<u32>;
    data temp_forces: Vec<cell::Cell<coordinate::Coordinate>>;
    def __new__(_cls, somite_number: usize) -> PyResult<Caterpillar> {
        let somites = (0..somite_number).map(|i| {
            Somite::new_still_somite(
                CONFIG.somite_radius,
                coordinate::Coordinate{x: (i as f64)*2.*CONFIG.somite_radius, y: 0., z: CONFIG.somite_radius}
            )
        }).collect::<Vec<somite::Somite>>();

        let simulation_protocol = simulation_export::SimulationProc::new(
            somites.iter().enumerate().map(|(i, s)| {
                simulation_export::Object{id: format!("_somite_{}", i), rad: CONFIG.somite_radius, pos: s.get_position().to_tuple()}
            }).collect::<Vec<simulation_export::Object>>()
        );

        let temp_forces = (0..somite_number).map(|i| {
            cell::Cell::new(coordinate::Coordinate::zero())
        }).collect();

        Caterpillar::create_instance(
            py,
            somites,
            simulation_protocol,
            cell::Cell::<u32>::new(0),
            temp_forces,
        )
    }
    def show_positions(&self) -> PyResult<PyString> {
        let mut position_report = "Positions of somites\n".to_string();
        for s in self.somites(py) {
            position_report.push_str(&format!("{}\n", s.to_string()))
        }
        Ok(PyString::new(py, &position_report))
    }
    def set_positions(&self, somite: usize, x: f64, y: f64, z: f64) -> PyResult<PyObject> {
        self.somites(py)[somite].set_position(coordinate::Coordinate{x:x, y:y, z:z});
        Ok(py.None())
    }
    def print_config(&self) -> PyResult<String> {
        Ok(CONFIG.to_string())
    }
    def center_of_mass(&self) -> PyResult<PyTuple> {
        let center = self.calculate_center_of_mass(py);
        Ok(center.to_tuple().to_py_object(py))
    }
    def step(&self) -> PyResult<PyObject> {
        self.update_state(py);
        Ok(py.None())
    }
    def save_simulation(&self, file_path: String) -> PyResult<PyObject> {
        self.simulation_protocol(py).save(&file_path);
        Ok(py.None())
    }
    def set_force_on_somite(&self, somite_number: usize, force: (f64, f64, f64)) -> PyResult<PyObject> {
        self.temp_forces(py)[somite_number].set(
            self.temp_forces(py)[somite_number].get() + coordinate::Coordinate::from_tuple(force));
        Ok(py.None())
    }
});

impl Caterpillar {
    fn calculate_center_of_mass(&self, py: Python) -> coordinate::Coordinate {
        let mut sum = coordinate::Coordinate {
            x: 0.,
            y: 0.,
            z: 0.,
        };
        for s in self.somites(py) {
            sum += s.position.get();
        }
        sum / (self.somites(py).len() as f64)
    }

    fn update_state(&self, py: Python) {
        self.update_somite_positions(py);
        let new_forces = self.calculate_force_on_somites(py);
        self.update_somite_verocities(py, &new_forces);
        self.update_somite_forces(py, &new_forces);

        let decimation_span: u32 = 10;
        if self.frame_count(py).get() % decimation_span == (0 as u32) {
            // save the step into simulation protocol
            self.simulation_protocol(py).add_frame(
                self.frame_count(py).get() / decimation_span,
                self.build_current_frame(py),
            );
        }
        self.frame_count(py).set(self.frame_count(py).get() + 1);
    }

    fn build_current_frame(&self, py: Python) -> Vec<simulation_export::ObjectPosition> {
        self.somites(py)
            .into_iter()
            .enumerate()
            .map(|(i, s)| simulation_export::ObjectPosition {
                id: format!("_somite_{}", i),
                pos: s.get_position().to_tuple(),
            })
            .collect()
    }

    fn update_somite_positions(&self, py: Python) {
        // update somite positions based on Velret's method
        // x_{t+1} = x_{t} + \delta t v_{t} + 0.5 \delta t^2 f_{t, x_t}
        for s in self.somites(py) {
            let new_position = s.get_position() + s.get_verocity() * CONFIG.time_delta
                + s.get_force() * 0.5 * CONFIG.time_delta.powi(2);
            s.set_position(new_position);
        }
    }

    fn update_somite_verocities(&self, py: Python, new_forces: &Vec<coordinate::Coordinate>) {
        // update somite verocities based on Velret's method
        // v_{t+1} = v_{t} +  \delta \frac{t (f_{t, x_t} + f_{t+1, x_{t+1}})}{2}
        for (i, s) in self.somites(py).iter().enumerate() {
            let new_verocity =
                s.get_verocity() + (s.get_force() + new_forces[i]) * 0.5 * CONFIG.time_delta;
            s.set_verocity(new_verocity);
        }
    }

    fn update_somite_forces(&self, py: Python, new_forces: &Vec<coordinate::Coordinate>) {
        for (i, s) in self.somites(py).iter().enumerate() {
            s.set_force(new_forces[i]);
        }
    }

    fn calculate_force_on_somites(&self, py: Python) -> Vec<coordinate::Coordinate> {
        let mut new_forces = self.temp_forces(py)
            .iter()
            .map(|f| f.replace(coordinate::Coordinate::zero()))
            .collect::<Vec<coordinate::Coordinate>>();

        // calculate resultant force from friction, tension, dumping, etc.
        for (i, s) in self.somites(py).iter().enumerate() {
            new_forces[i] += s.calculate_friction(CONFIG.friction_coeff)
        }

        // add spring and dumper effects
        let sp = spring::Spring::new(CONFIG.sp_k, CONFIG.sp_natural_length);
        let dp = dumper::Dumper::new(CONFIG.dp_c);
        for i in (0..(self.somites(py).len() - 1)) {
            new_forces[i] += sp.force(
                self.somites(py)[i + 1].get_position(),
                self.somites(py)[i].get_position(),
            );
            new_forces[i + 1] += sp.force(
                self.somites(py)[i].get_position(),
                self.somites(py)[i + 1].get_position(),
            );

            new_forces[i] += dp.force(
                self.somites(py)[i + 1].get_verocity(),
                self.somites(py)[i].get_verocity(),
            );
            new_forces[i + 1] += dp.force(
                self.somites(py)[i].get_verocity(),
                self.somites(py)[i + 1].get_verocity(),
            );
        }

        new_forces
    }
}

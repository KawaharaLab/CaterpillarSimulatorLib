#[macro_use]
extern crate cpython;

#[macro_use]
extern crate serde_derive;

use std::f64;
use std::cell;
use std::collections;
use cpython::{PyDict, PyObject, PyResult, PyString, PyTuple, Python, PythonObject, ToPyObject};

mod phase_oscillator;
mod somite;
mod torsion_spring;
mod spring;
mod dumper;
mod caterpillar_config;
mod coordinate;
mod simulation_export;

const GRAVITATIONAL_ACCELERATION: f64 = 9.8065;

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
    data config: caterpillar_config::Config;
    data somites: Vec<somite::Somite>;
    data simulation_protocol: simulation_export::SimulationProc;
    data frame_count: cell::Cell<u32>;
    data temp_forces: Vec<cell::Cell<coordinate::Coordinate>>;
    data oscillators: collections::HashMap<u32, cell::RefCell<phase_oscillator::PhaseOscillator>>;
    def __new__(_cls, somite_number: usize, somites_to_set_oscillater: &PyTuple, kwargs: Option<&PyDict>) -> PyResult<Caterpillar> {
        // parse config
        let config = match kwargs {
            Some(kwargs) => Self::parse_config(py, kwargs),
            None => caterpillar_config::Config::new(),
        };

        let somites = (0..somite_number).map(|i| {
            somite::Somite::new_still_somite(
                config.somite_radius,
                config.somite_mass,
                coordinate::Coordinate{x: (i as f64)*2.*config.somite_radius, y: 0., z: config.somite_radius}
            )
        }).collect::<Vec<somite::Somite>>();

        let simulation_protocol = simulation_export::SimulationProc::new(
            somites.iter().enumerate().map(|(i, s)| {
                simulation_export::Object{id: format!("_somite_{}", i), rad: config.somite_radius, pos: s.get_position().to_tuple()}
            }).collect::<Vec<simulation_export::Object>>()
        );

        let temp_forces = (0..somite_number).map(|_| {
            cell::Cell::new(coordinate::Coordinate::zero())
        }).collect();

        let mut oscillators = collections::HashMap::<u32, cell::RefCell<phase_oscillator::PhaseOscillator>>::new();
        for somite_id in somites_to_set_oscillater.iter(py) {
            oscillators.insert(somite_id.extract::<u32>(py).unwrap(), cell::RefCell::<phase_oscillator::PhaseOscillator>::new(phase_oscillator::PhaseOscillator::new()));
        }

        Caterpillar::create_instance(
            py,
            config,
            somites,
            simulation_protocol,
            cell::Cell::<u32>::new(0),
            temp_forces,
            oscillators,
        )
    }
    def print_config(&self) -> PyResult<PyString> {
        Ok(PyString::new(py, &format!("{}", self.config(py))))
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
    def center_of_mass(&self) -> PyResult<PyTuple> {
        let center = self.calculate_center_of_mass(py);
        Ok(center.to_tuple().to_py_object(py))
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
    def step(&self, dt: f64) -> PyResult<PyObject> {
        for (_, oscillator) in self.oscillators(py) {
            oscillator.borrow_mut().step(self.config(py).normal_angular_velocity, dt);
        }
        self.update_state(py, dt);
        Ok(py.None())
    }
    def step_with_feedbacks(&self, dt: f64, feedbacks: PyTuple) -> PyResult<PyObject> {
        if feedbacks.len(py) != self.oscillators(py).len() {
            panic!("number of elements in feedbacks and oscillator controllers are inconsistent");
        }
        let mut iter = feedbacks.iter(py);
        for (_, oscillator) in self.oscillators(py) {
            oscillator.borrow_mut().step(self.config(py).normal_angular_velocity + iter.next().unwrap().extract::<f64>(py).unwrap(), dt);
        }

        self.update_state(py, dt);
        Ok(py.None())
    }
    def frictions_x(&self) -> PyResult<PyTuple> {
        Ok(
            PyTuple::new(
                py,
                self.somites(py).iter().map(|s| {
                    (s.get_verocity().x * -self.config(py).friction_coeff).into_py_object(py).into_object()
                }).collect::<Vec<PyObject>>().as_slice()
            )
        )
    }
    def tensions(&self) -> PyResult<PyTuple> {
        let sp = spring::Spring::new(self.config(py).sp_k, self.config(py).sp_natural_length);
        Ok(
            PyTuple::new(
                py,
                (0..(self.somites(py).len() - 1)).map(|i|{
                    sp.force(
                        self.somites(py)[i].get_position(),
                        self.somites(py)[i + 1].get_position(),
                    ).norm().into_py_object(py).into_object()
                }).collect::<Vec<PyObject>>().as_slice(),
            )
        )
    }
    def phases(&self) -> PyResult<PyTuple> {
        Ok(
            PyTuple::new(
                py,
                self.oscillators(py).values().map(|o| {
                    o.borrow().get_phase().into_py_object(py).into_object()
                }).collect::<Vec<PyObject>>().as_slice()
            )
        )
    }
});

impl Caterpillar {
    fn parse_config(py: Python, kwargs: &PyDict) -> caterpillar_config::Config {
        let mut config = caterpillar_config::Config::new();
        for (key, val) in kwargs.items(py) {
            config.set(
                key.extract::<String>(py).unwrap().as_ref(),
                val.extract::<f64>(py).unwrap(),
            );
        }
        config
    }

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

    fn update_state(&self, py: Python, time_delta: f64) {
        self.update_somite_positions(py, time_delta);
        let new_forces = self.calculate_force_on_somites(py);
        self.update_somite_verocities(py, time_delta, &new_forces);
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

    fn update_somite_positions(&self, py: Python, time_delta: f64) {
        // update somite positions based on Velret's method
        // x_{t+1} = x_{t} + \delta t v_{t} + 0.5 \delta t^2 f_{t, x_t}
        for s in self.somites(py) {
            let new_position = s.get_position() + s.get_verocity() * time_delta
                + s.get_force() * 0.5 * time_delta.powi(2) / s.mass;
            s.set_position(new_position);
        }
    }

    fn update_somite_verocities(
        &self,
        py: Python,
        time_delta: f64,
        new_forces: &Vec<coordinate::Coordinate>,
    ) {
        // update somite verocities based on Velret's method
        // v_{t+1} = v_{t} +  \delta \frac{t (f_{t, x_t} + f_{t+1, x_{t+1}})}{2}
        for (i, s) in self.somites(py).iter().enumerate() {
            let mut new_verocity =
                s.get_verocity() + (s.get_force() + new_forces[i]) * 0.5 * time_delta / s.mass;
            if s.is_on_ground() {
                new_verocity.z = new_verocity.z.max(0.);
            }
            s.set_verocity(new_verocity);
        }
    }

    fn update_somite_forces(&self, py: Python, new_forces: &Vec<coordinate::Coordinate>) {
        for (i, s) in self.somites(py).iter().enumerate() {
            s.set_force(new_forces[i]);
        }
    }

    fn calculate_force_on_somites(&self, py: Python) -> Vec<coordinate::Coordinate> {
        // calculate force from friction, tension, dumping, etc.
        // collect temporary force applied on somites and reset temp_forces instance variable
        let mut new_forces = self.temp_forces(py)
            .iter()
            .map(|f| f.replace(coordinate::Coordinate::zero()))
            .collect::<Vec<coordinate::Coordinate>>();

        // gravity force
        for (i, s) in self.somites(py).iter().enumerate() {
            new_forces[i].z += -GRAVITATIONAL_ACCELERATION * s.mass
        }

        // frictional force against ground
        for (i, s) in self.somites(py).iter().enumerate() {
            if s.is_on_ground() {
                new_forces[i].x += s.get_verocity().x * -self.config(py).friction_coeff;
                new_forces[i].y += s.get_verocity().y * -self.config(py).friction_coeff;
            }
        }

        // spring and dumper effects
        let sp = spring::Spring::new(self.config(py).sp_k, self.config(py).sp_natural_length);
        let dp = dumper::Dumper::new(self.config(py).dp_c);
        for i in 0..(self.somites(py).len() - 1) {
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

        // torsion spring
        let vertical_ts = torsion_spring::TorsionSpring::new(
            self.config(py).vertical_ts_k,
            coordinate::Coordinate {
                x: 0.,
                y: 1.,
                z: 0.,
            },
        );
        let horizon_ts = torsion_spring::TorsionSpring::new(
            self.config(py).horizon_ts_k,
            coordinate::Coordinate {
                x: 0.,
                y: 0.,
                z: 1.,
            },
        );
        for i in 0..(self.somites(py).len() - 2) {
            // torsion spring at i+1 th somite
            let target_angle = match self.oscillators(py).get(&(i as u32 + 1)) {
                Some(oscillator) => {
                    self.phase2torsion_spring_target_angle(py, oscillator.borrow().get_phase())
                }
                None => 0.,
            };

            // vertical torsion spring
            new_forces[i] += vertical_ts.force(
                self.somites(py)[i + 2].get_position(),
                self.somites(py)[i + 1].get_position(),
                self.somites(py)[i].get_position(),
                target_angle,
            );

            new_forces[i + 1] -= vertical_ts.force(
                self.somites(py)[i + 2].get_position(),
                self.somites(py)[i + 1].get_position(),
                self.somites(py)[i].get_position(),
                target_angle,
            );

            new_forces[i + 2] += vertical_ts.force(
                self.somites(py)[i].get_position(),
                self.somites(py)[i + 1].get_position(),
                self.somites(py)[i + 2].get_position(),
                -target_angle,
            );

            new_forces[i + 1] -= vertical_ts.force(
                self.somites(py)[i].get_position(),
                self.somites(py)[i + 1].get_position(),
                self.somites(py)[i + 2].get_position(),
                -target_angle,
            );

            // horizon torsion spring
            new_forces[i] += horizon_ts.force(
                self.somites(py)[i + 2].get_position(),
                self.somites(py)[i + 1].get_position(),
                self.somites(py)[i].get_position(),
                0.,
            );

            new_forces[i + 1] -= horizon_ts.force(
                self.somites(py)[i + 2].get_position(),
                self.somites(py)[i + 1].get_position(),
                self.somites(py)[i].get_position(),
                0.,
            );

            new_forces[i + 2] += horizon_ts.force(
                self.somites(py)[i].get_position(),
                self.somites(py)[i + 1].get_position(),
                self.somites(py)[i + 2].get_position(),
                0.,
            );

            new_forces[i + 1] -= horizon_ts.force(
                self.somites(py)[i].get_position(),
                self.somites(py)[i + 1].get_position(),
                self.somites(py)[i + 2].get_position(),
                0.,
            );
        }

        // mask negative z force is a somite is on ground
        self.mask_force_on_landing(py, new_forces)
    }

    fn mask_force_on_landing(
        &self,
        py: Python,
        mut forces: Vec<coordinate::Coordinate>,
    ) -> Vec<coordinate::Coordinate> {
        for (i, s) in self.somites(py).iter().enumerate() {
            if s.is_on_ground() {
                forces[i].z = forces[i].z.max(0.)
            }
        }
        forces
    }

    fn phase2torsion_spring_target_angle(&self, py: Python, phase: f64) -> f64 {
        self.config(py).realtime_tunable_ts_rom * (phase.cos() - 1.) * 0.5
    }
}
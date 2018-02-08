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

use coordinate::Coordinate;

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
    data temp_forces: Vec<cell::Cell<Coordinate>>;
    data oscillators: collections::HashMap<u32, cell::RefCell<phase_oscillator::PhaseOscillator>>;
    data frictional_forces: Vec<cell::Cell<Coordinate>>;
    data target_angles: cell::RefCell<collections::HashMap<u32, f64>>;
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
                Coordinate{x: (i as f64)*2.*config.somite_radius, y: 0., z: config.somite_radius}
            )
        }).collect::<Vec<somite::Somite>>();

        let simulation_protocol = simulation_export::SimulationProc::new(
            somites.iter().enumerate().map(|(i, s)| {
                simulation_export::Object{id: format!("_somite_{}", i), rad: config.somite_radius, pos: s.get_position().to_tuple()}
            }).collect::<Vec<simulation_export::Object>>()
        );

        let temp_forces = (0..somite_number).map(|_| {
            cell::Cell::new(Coordinate::zero())
        }).collect();

        let mut oscillators = collections::HashMap::<u32, cell::RefCell<phase_oscillator::PhaseOscillator>>::new();
        for somite_id in somites_to_set_oscillater.iter(py) {
            oscillators.insert(
                somite_id.extract::<u32>(py).unwrap(),
                cell::RefCell::<phase_oscillator::PhaseOscillator>::new(phase_oscillator::PhaseOscillator::new())
            );
        }

        let initial_frictions = (0..somite_number).map(|_| {
            cell::Cell::new(Coordinate::zero())
        }).collect();

        let target_angles = cell::RefCell::<collections::HashMap<u32, f64>>::new(collections::HashMap::<u32, f64>::new());

        Caterpillar::create_instance(
            py,
            config,
            somites,
            simulation_protocol,
            cell::Cell::<u32>::new(0),
            temp_forces,
            oscillators,
            initial_frictions,
            target_angles,
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
        self.somites(py)[somite].set_position(Coordinate{x:x, y:y, z:z});
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
            self.temp_forces(py)[somite_number].get() + Coordinate::from_tuple(force));
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
    def step_with_target_angles(&self, dt: f64, target_angles: PyTuple) -> PyResult<PyObject> {
        if target_angles.len(py) != self.somites(py).len() - 2 {
            panic!("number of elements in target_angles and torsion springs are inconsistent");
        }
        for (i, target_angle) in target_angles.iter(py).enumerate() {
            self.target_angles(py).borrow_mut().insert(i as u32 + 1, target_angle.extract::<f64>(py).unwrap());
        }

        self.update_state(py, dt);
        Ok(py.None())
    }
    def frictions_x(&self) -> PyResult<PyTuple> {
        Ok(
            PyTuple::new(
                py,
                self.frictional_forces(py).iter().map(|f| {
                    f.get().x.into_py_object(py).into_object()
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

    fn calculate_center_of_mass(&self, py: Python) -> Coordinate {
        let mut sum = Coordinate::new(0., 0., 0.);
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

    fn update_somite_verocities(&self, py: Python, time_delta: f64, new_forces: &Vec<Coordinate>) {
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

    fn update_somite_forces(&self, py: Python, new_forces: &Vec<Coordinate>) {
        for (i, s) in self.somites(py).iter().enumerate() {
            s.set_force(new_forces[i]);
        }
    }

    fn calculate_force_on_somites(&self, py: Python) -> Vec<Coordinate> {
        // calculate force from friction, tension, dumping, etc.
        // collect temporary force applied on somites and reset temp_forces instance variable
        let mut new_forces = self.temp_forces(py)
            .iter()
            .map(|f| f.replace(Coordinate::zero()))
            .collect::<Vec<Coordinate>>();
        new_forces = self.add_gravitational_forces(py, new_forces);
        new_forces = self.add_spring_forces(
            py,
            self.config(py).sp_k,
            self.config(py).sp_natural_length,
            new_forces,
        );
        new_forces = self.add_dumper_forces(py, self.config(py).dp_c, new_forces);

        let horizon_ts = torsion_spring::TorsionSpring::new(
            self.config(py).horizon_ts_k,
            Coordinate::new(0., 0., 1.),
        );
        let horizon_angles = (1..self.somites(py).len() - 1)
            .map(|_| 0.)
            .collect::<Vec<f64>>();
        new_forces = self.add_torsion_spring_forces(py, horizon_ts, horizon_angles, new_forces);

        let vertical_ts = torsion_spring::TorsionSpring::new(
            self.config(py).vertical_ts_k,
            Coordinate::new(0., 1., 0.),
        );
        let vertical_angles = (1..self.somites(py).len() - 1)
            .map(
                |i| match self.target_angles(py).borrow_mut().remove(&(i as u32)) {
                    Some(target_angle) => target_angle,
                    None => match self.oscillators(py).get(&(i as u32)) {
                        Some(oscillator) => self.phase2torsion_spring_target_angle(
                            py,
                            oscillator.borrow().get_phase(),
                        ),
                        None => 0.,
                    },
                },
            )
            .collect::<Vec<f64>>();
        new_forces = self.add_torsion_spring_forces(py, vertical_ts, vertical_angles, new_forces);
        new_forces = self.add_frictional_forces(py, new_forces);

        self.mask_force_on_landing(py, new_forces)
    }

    fn mask_force_on_landing(&self, py: Python, mut forces: Vec<Coordinate>) -> Vec<Coordinate> {
        // mask negative z force if a somite is on ground
        // this process should be the very end of resultant force calculation
        for (i, s) in self.somites(py).iter().enumerate() {
            if s.is_on_ground() {
                forces[i].z = forces[i].z.max(0.)
            }
        }
        forces
    }

    fn add_gravitational_forces(&self, py: Python, mut forces: Vec<Coordinate>) -> Vec<Coordinate> {
        for (i, s) in self.somites(py).iter().enumerate() {
            forces[i].z += -GRAVITATIONAL_ACCELERATION * s.mass
        }
        forces
    }

    fn add_spring_forces(
        &self,
        py: Python,
        spring_constant: f64,
        natural_length: f64,
        mut forces: Vec<Coordinate>,
    ) -> Vec<Coordinate> {
        let sp = spring::Spring::new(spring_constant, natural_length);
        for i in 0..(self.somites(py).len() - 1) {
            forces[i] += sp.force(
                self.somites(py)[i + 1].get_position(),
                self.somites(py)[i].get_position(),
            );
            forces[i + 1] += sp.force(
                self.somites(py)[i].get_position(),
                self.somites(py)[i + 1].get_position(),
            );
        }
        forces
    }

    fn add_dumper_forces(
        &self,
        py: Python,
        dumping_coeff: f64,
        mut forces: Vec<Coordinate>,
    ) -> Vec<Coordinate> {
        let dp = dumper::Dumper::new(dumping_coeff);
        for i in 0..(self.somites(py).len() - 1) {
            forces[i] += dp.force(
                self.somites(py)[i + 1].get_verocity(),
                self.somites(py)[i].get_verocity(),
            );
            forces[i + 1] += dp.force(
                self.somites(py)[i].get_verocity(),
                self.somites(py)[i + 1].get_verocity(),
            );
        }
        forces
    }

    fn add_torsion_spring_forces(
        &self,
        py: Python,
        t_spring: torsion_spring::TorsionSpring,
        target_angles: Vec<f64>,
        mut forces: Vec<Coordinate>,
    ) -> Vec<Coordinate> {
        // target_angles[i-1] corresponds to a torsion spring on somite i
        if target_angles.len() != self.somites(py).len() - 2 {
            panic!(
                "target_angles should be somites.len() - 2 = {}, got {}",
                self.somites(py).len() - 2,
                target_angles.len()
            );
        }
        for i in 1..(self.somites(py).len() - 1) {
            let b_pos = self.somites(py)[i - 1].get_position();
            let m_pos = self.somites(py)[i].get_position();
            let f_pos = self.somites(py)[i + 1].get_position();
            let target_arg = target_angles[i - 1];

            // torsion spring at i+1 th somite
            forces[i - 1] += t_spring.force(f_pos, m_pos, b_pos, -target_arg);
            forces[i] -= t_spring.force(f_pos, m_pos, b_pos, -target_arg)
                + t_spring.force(b_pos, m_pos, f_pos, target_arg);
            forces[i + 1] += t_spring.force(b_pos, m_pos, f_pos, target_arg);
        }
        forces
    }

    fn add_frictional_forces(&self, py: Python, mut forces: Vec<Coordinate>) -> Vec<Coordinate> {
        for (i, s) in self.somites(py).iter().enumerate() {
            if s.is_on_ground() {
                let friction = friction(
                    self.config(py).static_friction_coeff,
                    self.config(py).dynamic_friction_coeff,
                    self.config(py).viscosity_friction_coeff,
                    s,
                    &forces[i],
                );
                self.frictional_forces(py)[i].set(friction);
                forces[i] += friction;
            }
        }
        forces
    }

    fn phase2torsion_spring_target_angle(&self, py: Python, phase: f64) -> f64 {
        self.config(py).realtime_tunable_ts_rom * (1. - phase.cos()) * 0.5
    }
}

fn friction(
    static_friction_coeff: f64,
    dynamic_friction_coeff: f64,
    viscosity_friction_coeff: f64,
    somite: &somite::Somite,
    force: &Coordinate,
) -> Coordinate {
    // frictional force should depend on negative force along z-axis
    // static friction or viscosity + dynamic friction
    let verocity = somite.get_verocity();
    let friction_x = if somite.is_moving_x() {
        // dynamic frictional force + viscosity
        dynamic_friction_coeff * force.z.min(0.) * &somite.get_verocity_direction_x()
            + -viscosity_friction_coeff * verocity.x
    } else {
        // static frictional force
        if force.x.abs() > static_friction_coeff * force.z.min(0.).abs() {
            force.x.signum() * static_friction_coeff * force.z.min(0.)
        } else {
            -force.x
        }
    };

    let friction_y = if somite.is_moving_y() {
        // dynamic frictional force
        dynamic_friction_coeff * force.z.min(0.) * &somite.get_verocity_direction_y()
            + -viscosity_friction_coeff * verocity.y
    } else {
        // static frictional force
        if force.y.abs() > static_friction_coeff * force.z.min(0.).abs() {
            force.y.signum() * static_friction_coeff * force.z.min(0.)
        } else {
            -force.y
        }
    };

    Coordinate::new(friction_x, friction_y, 0.)
}

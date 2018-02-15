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
mod calculations;

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
    data frame_count: cell::Cell<usize>;
    data temp_forces: Vec<cell::Cell<Coordinate>>;
    data oscillators: collections::HashMap<usize, cell::RefCell<phase_oscillator::PhaseOscillator>>;
    data oscillator_ids: Vec<usize>;
    data gripping_oscillators: collections::HashMap<usize, cell::RefCell<phase_oscillator::PhaseOscillator>>;
    data gripping_oscillator_ids: Vec<usize>;
    data oscillation_ranges: collections::HashMap<usize, cell::Cell<f64>>;
    data frictional_forces: Vec<cell::Cell<Coordinate>>;
    data target_angles: cell::RefCell<collections::HashMap<usize, f64>>;
    data torsion_spring_tensions: Vec<cell::Cell<f64>>;
    data gripping_thresholds: collections::HashMap<usize, cell::Cell<f64>>;
    data previous_vertical_torsion_spring_angles: collections::HashMap<usize, cell::Cell<f64>>;
    def __new__(
        _cls,
        somite_number: usize,
        somites_to_set_oscillater: &PyTuple,
        somites_to_set_gripper: &PyTuple,
        kwargs: Option<&PyDict>
    ) -> PyResult<Caterpillar> {
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

        let mut oscillators = collections::HashMap::<usize, cell::RefCell<phase_oscillator::PhaseOscillator>>::new();
        for somite_id in somites_to_set_oscillater.iter(py) {
            let id = somite_id.extract::<usize>(py).unwrap();
            oscillators.insert(
                id,
                cell::RefCell::<phase_oscillator::PhaseOscillator>::new(phase_oscillator::PhaseOscillator::new())
            );
        }
        let mut oscillator_ids = oscillators.keys().map(|k| {k.clone()}).collect::<Vec<usize>>();
        oscillator_ids.sort();

        let mut gripping_oscillators = collections::HashMap::<usize, cell::RefCell<phase_oscillator::PhaseOscillator>>::new();
        let mut gripping_thresholds = collections::HashMap::<usize, cell::Cell<f64>>::new();
        for somite_id in somites_to_set_gripper.iter(py) {
            let id = somite_id.extract::<usize>(py).unwrap();
            gripping_oscillators.insert(id, cell::RefCell::<phase_oscillator::PhaseOscillator>::new(phase_oscillator::PhaseOscillator::new()));
            gripping_thresholds.insert(id, cell::Cell::<f64>::new(config.gripping_phase_threshold));
        }
        let mut gripping_oscillator_ids = gripping_oscillators.keys().map(|k| {k.clone()}).collect::<Vec<usize>>();
        gripping_oscillator_ids.sort();

        let mut oscillation_ranges = collections::HashMap::<usize, cell::Cell<f64>>::new();
        for somite_id in somites_to_set_oscillater.iter(py) {
            oscillation_ranges.insert(
                somite_id.extract::<usize>(py).unwrap(),
                cell::Cell::<f64>::new(config.realtime_tunable_ts_rom)
            );
        }

        let initial_frictions = (0..somite_number).map(|_| {
            cell::Cell::new(Coordinate::zero())
        }).collect();

        let target_angles = cell::RefCell::<collections::HashMap<usize, f64>>::new(collections::HashMap::<usize, f64>::new());

        let tensions = (0..somite_number-2).map(|_| {
            cell::Cell::<f64>::new(0.)
        }).collect::<Vec<cell::Cell<f64>>>();

        let mut previous_vertical_torsion_spring_angles = collections::HashMap::<usize, cell::Cell<f64>>::new();
        for i in 1..somite_number-1 {
            previous_vertical_torsion_spring_angles.insert(i as usize, cell::Cell::<f64>::new(0.));
        }


        Caterpillar::create_instance(
            py,
            config,
            somites,
            simulation_protocol,
            cell::Cell::<usize>::new(0),
            temp_forces,
            oscillators,
            oscillator_ids,
            gripping_oscillators,
            gripping_oscillator_ids,
            oscillation_ranges,
            initial_frictions,
            target_angles,
            tensions,
            gripping_thresholds,
            previous_vertical_torsion_spring_angles,
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
    def somite_positions(&self) -> PyResult<PyTuple> {
        Ok(
            PyTuple::new(
                py,
                self.somites(py).iter().map(|s| {
                    s.get_position().to_tuple().into_py_object(py).into_object()
                }).collect::<Vec<PyObject>>().as_slice()
            )
        )
    }
    def set_positions(&self, somite: usize, x: f64, y: f64, z: f64) -> PyResult<PyObject> {
        self.somites(py)[somite].set_position(Coordinate{x:x, y:y, z:z});
        Ok(py.None())
    }
    def center_of_mass(&self) -> PyResult<PyTuple> {
        let center = self.calculate_center_of_mass(py);
        Ok(center.to_tuple().to_py_object(py))
    }
    def head_position(&self) -> PyResult<PyTuple> {
        Ok(self.somites(py).last().unwrap().get_position().to_tuple().to_py_object(py))
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
    def set_oscillation_ranges(&self, angle_ranges: PyTuple) -> PyResult<PyObject> {
        if angle_ranges.len(py) != self.oscillators(py).len() {
            panic!("number of elements in angle_ranges({}) and oscillator controllers({}) are inconsistent",
                angle_ranges.len(py), self.oscillators(py).len());
        }
        for (i, r) in angle_ranges.iter(py).enumerate() {
            self.oscillation_ranges(py).get(&i).unwrap().set(r.extract::<f64>(py).unwrap());
        }
        Ok(py.None())
    }
    def set_gripping_phase_thresholds(&self, phase_thresholds: PyTuple) -> PyResult<PyObject> {
        if phase_thresholds.len(py) != self.gripping_oscillators(py).len() {
            panic!("number of elements in phase_threshold({}) and gripping oscillator controllers({}) are inconsistent",
                phase_thresholds.len(py), self.gripping_oscillators(py).len());
        }
        for (i, th) in phase_thresholds.iter(py).enumerate() {
            self.gripping_thresholds(py).get(&i).unwrap().set(th.extract::<f64>(py).unwrap());
        }
        Ok(py.None())
    }
    def step(&self, dt: f64) -> PyResult<PyObject> {
        // update somites' oscillators
        for (_, oscillator) in self.oscillators(py) {
            oscillator.borrow_mut().step(self.config(py).normal_angular_velocity, dt);
        }
        // update grippers' oscillators
        for (_, oscillator) in self.gripping_oscillators(py) {
            oscillator.borrow_mut().step(self.config(py).normal_angular_velocity, dt);
        }
        self.update_state(py, dt);
        Ok(py.None())
    }
    def step_with_feedbacks(&self, dt: f64, feedbacks_somites: PyTuple, feedbacks_grippers: PyTuple) -> PyResult<PyObject> {
        if feedbacks_somites.len(py) != self.oscillators(py).len() {
            panic!("number of elements in feedbacks_somites and oscillator controllers for somites are inconsistent");
        }
        if feedbacks_grippers.len(py) != self.gripping_oscillators(py).len() {
            panic!("number of elements in feedbacks_grippers and oscillator controllers for grippers are inconsistent");
        }
        for (i, f) in feedbacks_somites.iter(py).enumerate() {
            self.oscillators(py)
                .get(&self.order2somite_oscillator_id(py, i))
                .unwrap()
                .borrow_mut()
                .step(self.config(py).normal_angular_velocity + f.extract::<f64>(py).unwrap(), dt);
        }
        for (i, f) in feedbacks_grippers.iter(py).enumerate() {
            self.gripping_oscillators(py)
            .get(&self.order2gripping_oscillator_id(py, i))
            .unwrap()
            .borrow_mut()
            .step(self.config(py).normal_angular_velocity + f.extract::<f64>(py).unwrap(), dt);
        }
        self.update_state(py, dt);
        Ok(py.None())
    }
    def step_with_target_angles(&self, dt: f64, somite_target_angles: PyTuple, gripper_target_angles: PyTuple) -> PyResult<PyObject> {
        if somite_target_angles.len(py) != self.somites(py).len() - 2 {
            panic!("number of elements in somite_target_angles and torsion springs are inconsistent");
        }
        if gripper_target_angles.len(py) != self.gripping_oscillators(py).len() {
            panic!("number of elements in gripper_target_angles and torsion springs are inconsistent");
        }
        for (i, target_angle) in somite_target_angles.iter(py).enumerate() {
            self.target_angles(py).borrow_mut().insert(i + 1, target_angle.extract::<f64>(py).unwrap());
        }
        let mut gripper_target_angles_iter = gripper_target_angles.iter(py);
        for i in 0..self.somites(py).len() {
            if let Some(oscillator) = self.gripping_oscillators(py).get(&i) {
                oscillator.borrow_mut().set_phase(gripper_target_angles_iter.next().unwrap().extract::<f64>(py).unwrap());
            }
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
        Ok(
            PyTuple::new(
                py,
                self.torsion_spring_tensions(py).iter().map(|tension| {
                    tension.get().into_py_object(py).into_object()
                }).collect::<Vec<PyObject>>().as_slice(),
            )
        )
    }
    def somite_phases(&self) -> PyResult<PyTuple> {
        Ok(
            PyTuple::new(
                py,
                self.oscillator_ids(py).iter().map(|id| {
                    self.oscillators(py).get(&id).unwrap().borrow().get_phase().into_py_object(py).into_object()
                }).collect::<Vec<PyObject>>().as_slice()
            )
        )
    }
    def gripper_phases(&self) -> PyResult<PyTuple> {
        Ok(
            PyTuple::new(
                py,
                self.gripping_oscillator_ids(py).iter().map(|id| {
                    self.gripping_oscillators(py).get(&id).unwrap().borrow().get_phase().into_py_object(py).into_object()
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
        let new_forces = self.calculate_force_on_somites(py, time_delta);
        self.update_somite_verocities(py, time_delta, &new_forces);
        self.update_somite_forces(py, &new_forces);

        let decimation_span = 10_usize;
        if self.frame_count(py).get() % decimation_span == 0_usize {
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

    fn calculate_force_on_somites(&self, py: Python, time_delta: f64) -> Vec<Coordinate> {
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

        // vertical torsion force comming from material mechanical
        let vertical_ts = torsion_spring::TorsionSpring::new(
            self.config(py).vertical_ts_k0,
            self.config(py).vertical_ts_k1,
            Coordinate::new(0., 1., 0.),
        );
        let zero_angles = (1..self.somites(py).len() - 1)
            .map(|_| 0.)
            .collect::<Vec<f64>>();
        new_forces =
            self.add_torsion_spring_forces(py, vertical_ts, zero_angles, time_delta, new_forces);

        // vertical torsion force comming from actuator
        let vertical_realtime_tunable_ts = torsion_spring::TorsionSpring::new(
            self.config(py).vertical_realtime_tunable_torsion_spirng_k,
            0.,
            Coordinate::new(0., 1., 0.),
        );
        let vertical_discrepancy_angles = (1..self.somites(py).len() - 1)
            .map(|i| {
                // i is somite id
                let current_angle = vertical_realtime_tunable_ts.current_angle(
                    self.somites(py)[i - 1].get_position(),
                    self.somites(py)[i].get_position(),
                    self.somites(py)[i + 1].get_position(),
                );
                match self.target_angles(py).borrow_mut().remove(&i) {
                    Some(target_angle) => (target_angle - current_angle).max(0.),
                    None => match self.oscillators(py).get(&i) {
                        Some(oscillator) => {
                            let target_angle = phase2torsion_spring_target_angle(
                                oscillator.borrow().get_phase(),
                                self.oscillation_ranges(py).get(&i).unwrap().get(),
                            );
                            target_angle - current_angle
                        }
                        None => 0.,
                    },
                }
            })
            .collect::<Vec<f64>>();
        let torsion_spring_tensions = self.calculate_realtime_tunable_torsion_spring_tensions(
            py,
            &vertical_realtime_tunable_ts,
            &vertical_discrepancy_angles,
        );
        for (i, tension) in torsion_spring_tensions.into_iter().enumerate() {
            self.torsion_spring_tensions(py)[i].set(tension);
        }
        new_forces = self.add_realtime_tunable_torsion_spring_forces(
            py,
            vertical_realtime_tunable_ts,
            vertical_discrepancy_angles,
            new_forces,
        );

        // new_forces = self.add_frictional_forces(py, new_forces);
        new_forces = self.add_gripping_forces(py, new_forces);

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
        time_delta: f64,
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
            // calculate dumping torque
            let angle = t_spring.current_angle(
                self.somites(py)[i - 1].get_position(),
                self.somites(py)[i].get_position(),
                self.somites(py)[i + 1].get_position(),
            );
            let angular_velocity = calculations::differentiate(
                self.previous_vertical_torsion_spring_angles(py)
                    .get(&i)
                    .unwrap()
                    .get(),
                angle,
                time_delta,
            ).unwrap();
            let dumping_coeff = self.config(py).vertical_ts_c;
            let dumping_torque = -dumping_coeff * angular_velocity; // anti-clock-wise is positive rotation

            // torsion spring at i+1 th somite
            let (force_on_t, force_on_b) = t_spring.force(
                self.somites(py)[i - 1].get_position(),
                self.somites(py)[i].get_position(),
                self.somites(py)[i + 1].get_position(),
                target_angles[i - 1],
                dumping_torque,
            );
            forces[i - 1] += force_on_b;
            forces[i] -= force_on_b + force_on_t; // reaction
            forces[i + 1] += force_on_t;

            // memorize current angle
            self.previous_vertical_torsion_spring_angles(py)
                .get(&i)
                .unwrap()
                .set(angle);
        }
        forces
    }

    fn add_realtime_tunable_torsion_spring_forces(
        &self,
        py: Python,
        t_spring: torsion_spring::TorsionSpring,
        discrepancy_angles: Vec<f64>,
        mut forces: Vec<Coordinate>,
    ) -> Vec<Coordinate> {
        // discrepancy_angles[i-1] corresponds to a torsion spring on somite i
        let mut discrepancy_angles_iter = discrepancy_angles.into_iter();
        for i in 1..(self.somites(py).len() - 1) {
            // torsion spring at i+1 th somite
            let (force_on_t, force_on_b) = t_spring.force_on_discrepancy(
                self.somites(py)[i - 1].get_position(),
                self.somites(py)[i].get_position(),
                self.somites(py)[i + 1].get_position(),
                discrepancy_angles_iter.next().unwrap(),
            );
            forces[i - 1] += force_on_b;
            forces[i] -= force_on_b + force_on_t; // reaction
            forces[i + 1] += force_on_t;
        }
        if let Some(_) = discrepancy_angles_iter.next() {
            panic!("too many elements in discrepancy_angles");
        }
        forces
    }

    fn calculate_realtime_tunable_torsion_spring_tensions(
        &self,
        py: Python,
        t_spring: &torsion_spring::TorsionSpring,
        discrepancy_angles: &Vec<f64>,
    ) -> Vec<f64> {
        // tension i is force applied to torsion spring on i - 1 th somite
        if discrepancy_angles.len() != self.somites(py).len() - 2 {
            panic!(
                "discrepancy_angle should be somites.len() - 2 = {}, got {}",
                self.somites(py).len() - 2,
                discrepancy_angles.len()
            );
        }
        let mut tensions = Vec::<f64>::with_capacity(self.somites(py).len() - 2);
        for i in 1..(self.somites(py).len() - 1) {
            // torsion spring at i+1 th somite
            let (force_on_t, _) = t_spring.force_on_discrepancy(
                self.somites(py)[i - 1].get_position(),
                self.somites(py)[i].get_position(),
                self.somites(py)[i + 1].get_position(),
                discrepancy_angles[i - 1],
            );
            tensions.push(discrepancy_angles[i - 1].signum() * force_on_t.norm());
        }
        tensions
    }

    fn add_gripping_forces(&self, py: Python, mut forces: Vec<Coordinate>) -> Vec<Coordinate> {
        for (i, s) in self.somites(py).iter().enumerate() {
            if let Some(oscillator) = self.gripping_oscillators(py).get(&i) {
                // update grip state
                if oscillator.borrow().get_phase().sin()
                    < self.gripping_thresholds(py).get(&i).unwrap().get()
                {
                    if s.is_on_ground() && !s.is_gripping() {
                        s.grip();
                    }
                } else {
                    if s.is_gripping() {
                        s.release();
                    }
                }

                // calculate grip force
                if s.is_gripping() {
                    let gripping_point = s.get_gripping_point().unwrap();
                    let friction_x = -self.config(py).gripping_shear_stress_k
                        * (s.get_position().x - gripping_point.x)
                        - self.config(py).gripping_shear_stress_c * s.get_verocity().x;
                    forces[i] += Coordinate::new(friction_x, 0., -1. * forces[i].z.max(0.)); // cancel force along positive z axis
                    self.frictional_forces(py)[i].set(Coordinate::new(friction_x, 0., 0.));
                } else {
                    if s.is_on_ground() {
                        let dynamic_friction_x = s.get_verocity().x.signum()
                            * self.config(py).dynamic_friction_coeff
                            * forces[i].z.min(0.)
                            - self.config(py).viscosity_friction_coeff * s.get_verocity().x;
                        forces[i] += Coordinate::new(dynamic_friction_x, 0., 0.);
                        self.frictional_forces(py)[i].set(Coordinate::new(
                            dynamic_friction_x,
                            0.,
                            0.,
                        ));
                    } else {
                        self.frictional_forces(py)[i].set(Coordinate::zero());
                    }
                }
            } else {
                // no gripper on the i th somite
                if s.is_on_ground() {
                    let dynamic_friction_x = s.get_verocity().x.signum()
                        * self.config(py).dynamic_friction_coeff
                        * forces[i].z.min(0.)
                        - self.config(py).viscosity_friction_coeff * s.get_verocity().x;
                    forces[i] += Coordinate::new(dynamic_friction_x, 0., 0.);
                    self.frictional_forces(py)[i].set(Coordinate::new(dynamic_friction_x, 0., 0.));
                } else {
                    self.frictional_forces(py)[i].set(Coordinate::zero());
                }
            }
        }
        forces
    }

    fn order2somite_oscillator_id(&self, py: Python, i: usize) -> usize {
        self.oscillator_ids(py)[i]
    }

    fn order2gripping_oscillator_id(&self, py: Python, i: usize) -> usize {
        self.gripping_oscillator_ids(py)[i]
    }
}

fn phase2torsion_spring_target_angle(phase: f64, range: f64) -> f64 {
    range * (1. - phase.cos()) * 0.5
}

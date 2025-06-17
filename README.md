# CaterpillarSimulatorLib

A high-performance 3D caterpillar physics simulation library for Python, implemented in Rust. This library provides a realistic biomechanical simulation of caterpillar locomotion with support for reinforcement learning experiments and custom control algorithms.

## Features

- **High-Performance Physics Engine**: Rust-based implementation for fast simulation
- **Realistic Biomechanics**: Accurate modeling of caterpillar body segments, springs, and friction
- **Reinforcement Learning Support**: Built-in interfaces for training control policies
- **3D Visualization**: Blender integration for rendering simulation results
- **Flexible Control**: Support for both direct control and oscillator-based patterns
- **Extensible Architecture**: Easy to add custom controllers and training algorithms

## Requirements

- Python 3.6+
- Rust and Cargo (for building from source)
- setuptools-rust
- NumPy
- Blender (optional, for visualization)

## Installation

### Development Mode (Recommended)

```bash
git clone https://github.com/KawaharaLab/CaterpillarSimulatorLib.git
cd CaterpillarSimulatorLib
pip install -e .
```

If you're using pyenv:

```bash
pyenv exec pip install -e .
```

Note: You may need to use `sudo` depending on your Python installation.

### Building Requirements

Make sure you have Rust installed:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
```

## Quick Start

### Basic Usage

```python
import numpy as np
from caterpillar_lib import caterpillar

# Define caterpillar parameters
caterpillar_params = {
    "somite_mass": 0.3,
    "somite_radius": 0.35,
    "normal_angular_velocity": np.pi,
    "sp_natural_length": 0.7,
    "sp_k": 80.0,
    "dp_c": 10.0,
    "static_friction_coeff": 1.0,
    "dynamic_friction_coeff": 0.1,
    "viscosity_friction_coeff": 3.0,
}

# Create caterpillar with 5 segments
# Oscillators on segments (1,2,3) for movement control
c = caterpillar.Caterpillar(5, (1,2,3), (0,1,2,3,4), caterpillar_params)

# Run simulation steps
for _ in range(1000):
    c.step_with_feedbacks(0.01, (0, 0, 0), (0, 0, 0, 0, 0))

# Save simulation results
c.save_simulation("simulation_result.json")
```

### Running the Demo

To try the simulator with a pre-configured example:

```bash
python scripts/example.py
```

This will generate `test_render.json` containing the simulation data.

### Visualization with Blender

To visualize the simulation results:

```bash
blender --python scripts/render.py test_render.json
```

## Project Structure

```
CaterpillarSimulatorLib/
├── caterpillar_lib/           # Python package
│   ├── __init__.py           # Package initialization
│   └── caterpillar           # Rust extension module (built)
├── lib/                      # Rust implementation
│   ├── Cargo.toml           # Rust dependencies
│   └── src/                 # Rust source code
│       ├── lib.rs           # Main library interface
│       ├── caterpillar_config.rs  # Configuration handling
│       ├── somite.rs        # Body segment implementation
│       ├── dynamics.rs      # Physics dynamics
│       ├── phase_oscillator.rs   # Oscillator patterns
│       └── ...              # Other physics modules
├── example/                  # Training and control examples
│   ├── caterpillar_runner.py    # Simulation runner
│   ├── caterpillar_trainer.py   # Training interface
│   └── controllers/         # Control algorithms
│       ├── base_actor.py    # Base controller class
│       ├── linear_actor.py  # Linear controller
│       ├── pepg_actor_manager.py # PEPG algorithm
│       └── config.py        # Training configuration
├── scripts/                 # Utility scripts
│   ├── example.py          # Basic demo
│   └── render.py           # Blender visualization
└── setup.py                # Package setup
```

## API Reference

### Caterpillar Class

The main simulation class that represents a caterpillar with configurable segments and control systems.

#### Constructor

```python
Caterpillar(somite_number, oscillator_segments, gripper_segments, parameters, heights=None)
```

- `somite_number`: Number of body segments
- `oscillator_segments`: Tuple of segment IDs with movement oscillators
- `gripper_segments`: Tuple of segment IDs with gripping capability
- `parameters`: Dictionary of physical parameters
- `heights`: Optional terrain height map

#### Key Methods

- `step(dt)`: Advance simulation by time step dt
- `step_with_feedbacks(dt, somite_feedbacks, gripper_feedbacks)`: Step with control inputs
- `step_with_target_angles(dt, target_angles, gripper_phases)`: Step with target angles
- `save_simulation(filename)`: Save simulation data to JSON file
- `head_position()`: Get position of the head segment
- `center_of_mass()`: Get center of mass position
- `frictions_x()`: Get friction forces along x-axis
- `tensions()`: Get internal tension forces
- `somite_phases()`: Get oscillator phases

### Physical Parameters

Key parameters for caterpillar physics:

- `somite_mass`: Mass of each body segment (kg)
- `somite_radius`: Radius of each segment (m)
- `sp_k`: Spring constant between segments
- `sp_natural_length`: Natural length of inter-segment springs
- `dp_c`: Damping coefficient
- `static_friction_coeff`: Static friction with ground
- `dynamic_friction_coeff`: Dynamic friction coefficient
- `normal_angular_velocity`: Base oscillation frequency

## Examples

### Reinforcement Learning Training

```python
from example.caterpillar_runner import run_caterpillar
from example.controllers.linear_actor import Actor

# Create and train a controller
actor = Actor()
distance = run_caterpillar(actor, "./results", steps=10000)
print(f"Distance traveled: {distance}")
```

### Custom Control Pattern

```python
# Create caterpillar
c = caterpillar.Caterpillar(8, (1,2,3,4,5,6), (), caterpillar_params)

# Implement crawling pattern
for cycle in range(10):
    # Extension phase
    for _ in range(100):
        c.step_with_target_angles(0.01, (0, 0, 0), (1, 1, -1, -1, -1, -1))
    
    # Contraction phase  
    for _ in range(100):
        c.step_with_target_angles(0.01, (0, 0, 0), (-1, -1, 1, 1, 1, 1))
```


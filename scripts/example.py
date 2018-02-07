import numpy as np
from caterpillar_lib import caterpillar

if __name__ == '__main__':
    caterpillar_params = {
        "somite_mass": .3,
        "somite_radius": .35,
        "normal_angular_velocity": np.pi,
        "sp_natural_length": 0.7,
        "sp_k": 80.0,
        "dp_c": 10.0,
        "horizon_ts_k": 10.,
        "vertical_ts_k": 80.,
        "realtime_tunable_ts_rom": np.pi / 6.,
        "friction_coeff": 10.0,
    }
    c = caterpillar.Caterpillar(5, (1,3), caterpillar_params)

    for _ in range(20):
        c.set_force_on_somite(0, (0, 0, 5))
        c.step(0.01)
    for _ in range(30):
        c.set_force_on_somite(0, (3, 0, 0))
        c.step(0.01)
    for _ in range(10):
        c.set_force_on_somite(0, (0, 0, 0))
        c.step(0.01)
    for _ in range(500):
        c.step(0.01)

    c.save_simulation("test_render.json")

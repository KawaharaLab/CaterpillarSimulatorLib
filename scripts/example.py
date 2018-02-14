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
        "horizon_ts_k0": 0.,
        "horizon_ts_k1": 0.,
        "vertical_ts_k0": 20.,
        "vertical_ts_k1": 0.,
        "vertical_ts_c": 1.,
        "vertical_realtime_tunable_torsion_spirng_k": 30.,
        "realtime_tunable_ts_rom": np.pi * 1. / 3.,
        "static_friction_coeff": 1.0,
        "dynamic_friction_coeff": 0.1,
        "viscosity_friction_coeff": 3.0,
        "tip_sub_static_friction_coeff": 0.1,
        "tip_sub_dynamic_friction_coeff": 0.01,
        "tip_sub_viscosity_friction_coeff": 1.0,
        "friction_switch_tan": np.tan(np.pi * 1. / 3.),
        # "gripping_phase_threshold": np.sin(np.pi * 5. / 4.),
        "gripping_phase_threshold": np.sin(np.pi * 4. / 4.),
        "gripping_shear_stress_k": 500.,
        "gripping_shear_stress_c": 10.,
    }

    c = caterpillar.Caterpillar(5, (1,2,3), (0,1,2,3,4), caterpillar_params)

    # inching
    for _ in range(75):
        c.step_with_feedbacks(0.01, (0, 0, 0), (-np.pi, -np.pi, -np.pi, -np.pi, -np.pi))
    for _ in range(1000):
        c.step_with_feedbacks(0.01, (0, 0, 0), (0, 0, 0, 0, 0))

    # crawling
    for _ in range(10):
        for _ in range(100):
            c.step_with_target_angles(0.01, (0, 0, 0), (1, 1, -1, -1, -1))
        for _ in range(100):
            c.step_with_target_angles(0.01, (np.pi*1/2, 0, 0), (1, 1, -1, -1, -1))
        for _ in range(100):
            c.step_with_target_angles(0.01, (np.pi*1/2, 0, 0), (-1, -1, -1, -1, -1))
        for _ in range(100):
            c.step_with_target_angles(0.01, (np.pi*1/2, 0, 0), (-1, -1, 1, 1, -1))
        for _ in range(100):
            c.step_with_target_angles(0.01, (0, np.pi*1/2, 0), (-1, -1, 1, 1, -1))
        for _ in range(100):
            c.step_with_target_angles(0.01, (0, np.pi*1/2, 0), (-1, -1, -1, -1, -1))
        for _ in range(100):
            c.step_with_target_angles(0.01, (0, np.pi*1/2, 0), (-1, -1, -1, -1, -1))
        for _ in range(100):
            c.step_with_target_angles(0.01, (0, 0, np.pi*1/2), (-1, -1, -1, -1, 1))
        for _ in range(100):
            c.step_with_target_angles(0.01, (0, 0, 0), (-1, -1, -1, -1, 1))


    c.save_simulation("test_render.json")

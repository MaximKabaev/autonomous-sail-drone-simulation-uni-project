import ode
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # Initial state vector
    # [x position (m), y position (m), orientation (rad), x velocity (m/s), y velocity (m/s), angular velocity (rad/s)]
    s0 = np.array([
                [0.0],
                [0.0],
                [np.pi/2],
                [0.0],
                [2.9],
                [0.0]])

    # course points
    course_params_list = [
        ode.course_parameters(0, 60, -45, 0),
        ode.course_parameters(60, 65, -22.5, 2.1),
        ode.course_parameters(65, 100, -22.5, 0)
    ]

    # Create the saildrone object
    saildrone = ode.saildrone(course_params_list=course_params_list)

    saildrone.simulate_course(0, 100, 0.05, s0)

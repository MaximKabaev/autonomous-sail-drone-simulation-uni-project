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
    

    # ---------- AUTONOMOUS MODE -----------

    waypoints = [
        (0, 175),
        (200, 350)
    ]
    
    controller = ode.AutonomousController(
        waypoints=waypoints,
        wind=(-6.7, 0),  # Use actual wind vector
        Kp_rudder=0.5,  # Higher gain for stability (tune this value)
        lookahead_distance=5
    )
    
    saildrone = ode.saildrone(controller=controller)
    
    saildrone.simulate_course(0, 0.1, s0)
    
    # ---------- MANUAL MODE -----------
    # course_params_list = [
    #     ode.course_parameters(0, 60, -45, 0),
    #     ode.course_parameters(60, 65, -22.5, 2.1),
    #     ode.course_parameters(65, 100, -22.5, 0)
    # ]
    # saildrone = ode.saildrone(course_params_list=course_params_list)
    # saildrone.simulate_course(0, 0.1, s0)

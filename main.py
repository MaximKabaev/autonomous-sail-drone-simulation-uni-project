import ode
import numpy as np
import matplotlib.pyplot as plt


def simulate(s0):
    [t,s] = ode.solve_ivp(0,30,0.1,s0) 

    plt.figure()
    plt.plot(s[0,:], s[1,:], label='Trajectory')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Saildrone Trajectory')
    plt.legend()
    plt.grid()
    plt.show()


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
    
    simulate(s0)

import ode
import numpy as np
import matplotlib.pyplot as plt


def simulate(s0):
    [t,s] = ode.solve_ivp(0,60,0.05,s0)

    plt.figure()
    plt.plot(s[0,:], s[1,:], label='Trajectory')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Saildrone Trajectory')
    plt.legend()
    plt.grid()
    plt.show()
    print('Last state: ' + str(s[:, -1]))


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

    # s0 = np.array([
    #             [3.19382608],
    #             [1.73610746e+02],
    #             [1.51902967],
    #             [1.29768089e-01],
    #             [2.85967604],
    #             [-9.03440094e-04]])
    
    simulate(s0)

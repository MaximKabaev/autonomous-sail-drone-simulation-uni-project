import numpy as np
import saildrone_hydro
import matplotlib.pyplot as plt

# def sail_angle_at_time(t):
#     """
#     Compute the sail angle at time t.
    
#     Parameters
#     ----------
#     t : float64
#         Time in seconds.
        
#     Returns
#     -------
#     sail_angle : float64
#         Sail angle in radians.
#     """
#     # THIS FUNCTION IS INCOMPLETE.
#     # ------------------------------------------------------------------------
#     sail_angle = np.clip(np.deg2rad(-45), -np.pi, np.pi)  # Placeholder value
#     return sail_angle

# def rudder_angle_at_time(t):
#     """
#     Compute the rudder angle at time t.
    
#     Parameters
#     ----------
#     t : float64
#         Time in seconds.
        
#     Returns
#     -------
#     rudder_angle : float64
#         Rudder angle in radians.
#     """
#     # THIS FUNCTION IS INCOMPLETE.
#     # ------------------------------------------------------------------------
#     rudder_angle = np.clip(np.deg2rad(0), -np.pi/6, np.pi/6)  # Placeholder value
#     return rudder_angle

# def wind_at_time(t):
#     """
#     Compute the wind velocity at time t.

#     Parameters
#     ----------
#     t : float64
#         Time in seconds.
        
#     Returns
#     -------
#     wind_vector : numpy.ndarray
#         wind velocity vector in m/s.
#     """
#     # THIS FUNCTION IS INCOMPLETE.
#     # ------------------------------------------------------------------------

#     #NOTE: will be calculated from just the angle of heading as
#     wind_velocity = np.array([-6.7, 0])  # Placeholder value
#     return wind_velocity



def state_deriv_drone(t,s):
    """
    Compute the derivative of the state vector for the saildrone.
    
    Parameters
    ----------
    t : float64
        Time in seconds.

    s : numpy.ndarray
        State vector with five elements: x-position in m, y-position in m,
        heading in radians, x-velocity in m/s, y-velocity in m/s and turn rate in rad/s.
    """


    M = 2500  # Mass of the saildrone in kg
    I = 10000  # Moment of inertia of the saildrone in kg*m^2
    rho = 1.225  # Density of air in kg/m^3
    A_sail = 15.0  # Sail area in m^2

    sail_angle = np.deg2rad(-45)
    rudder_angle = np.deg2rad(0)
    v_wind = np.array([-6.7, 0])

    wind_angle = np.arctan2(v_wind[1], v_wind[0])

    v_apparent_wind = v_wind - np.array([s[3], s[4]])
    aparent_wind_angle = np.arctan2(v_apparent_wind[1], v_apparent_wind[0])

    s_dot = [s[3],                     # dx/dt = vx
             s[4],                     # dy/dt = vy
             s[5],                     # dtheta/dt = omega
             0,                        # dvx/dt
             0,                        # dvy/dt
             0]                        # domega/dt

    force_hydro, torque_hydro = saildrone_hydro.get_vals(
        np.array([s[3], s[4]]), s[2], s[5], rudder_angle)

    attack_angle = s[2] + sail_angle - aparent_wind_angle
    
    c_d = 1 - np.cos(2*(attack_angle))
    c_l = 1.5*np.sin(2*(attack_angle) + 0.5 * np.sin(2*(attack_angle)))
    F_d = 0.5 * rho * A_sail * np.linalg.norm(v_apparent_wind)**2 * c_d
    F_l = 0.5 * rho * A_sail * np.linalg.norm(v_apparent_wind)**2 * c_l
    F_d_x = F_d * np.cos(aparent_wind_angle)
    F_d_y = F_d * np.sin(aparent_wind_angle)
    F_l_x = F_l * np.cos(aparent_wind_angle + np.pi/2)
    F_l_y = F_l * np.sin(aparent_wind_angle + np.pi/2)
    
    s_dot[3] = 1/M * (F_d_x + F_l_x + force_hydro[0])
    s_dot[4] = 1/M * (F_d_y + F_l_y + force_hydro[1])
    s_dot[5] = 1/I * (torque_hydro + (0.1 * np.cos(s[2])*(F_d_y + F_l_y) - 0.1 * np.sin(s[2])*(F_d_x + F_l_x)))

    return np.array(s_dot)


def runge_kutta4_step(t,dt,s):
    """
    Apply the classical 4th-order Runge-Kutta method for one time step.

    Parameters
    ----------
    t : float64
        Time.
    dt : float64
        Time step.
    z : numpy.ndarray
        State vector at the current time step.
        
    Returns
    -------
    znext : numpy.ndarray
        State vector at the next time step.
    """
    
    A = state_deriv_drone(t, s)
    B = state_deriv_drone(t + dt/2, s + dt/2 * A)
    C = state_deriv_drone(t + dt/2, s + dt/2 * B)
    D = state_deriv_drone(t + dt, s + dt * C)

    snext = s + (dt/6) * (A + 2*B + 2*C + D)

    return snext


def solve_ivp(t0,tmax,dt,s0):
    """
    Solve an initial value problem (IVP) for an ordinary differential equation (ODE)
    
    Parameters
    ----------
    t0 : float64
        Start time.
    tmax : float64
        end time.
    dt : float64
        Time step.
    s0 : numpy.ndarray
        Initial state vector

    Returns
    -------
    t : numpy.ndarray
        Time axis.
    s : numpy.ndarray
        Solution as a matrix of state vectors.
    """
    
    # Set initial conditions
    t = np.array([t0])
    s = s0
    
    # Continue stepping until the end time is exceeded
    n=0
    while t[n] <= tmax:
        # Increment by one time step and append to the time axis
        t = np.append(t,t[-1]+dt)

        # Obtain next state vector using one step of Runge-Kutta 4 method
        snext = runge_kutta4_step(t[n], dt, s[:,n])
        
        # Append to the solution
        snext = snext[:,np.newaxis]
        s = np.append(s,snext,axis=1)

        n = n+1

    return t, s
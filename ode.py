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

class course_parameters:
    """
    Defines sail and rudder angles for a specific time interval.
    """
    def __init__(self, start_time, end_time, sail_angle_deg, rudder_angle_deg):
        """
        Parameters
        ----------
        start_time : float
            Start time of this course segment in seconds.
        end_time : float
            End time of this course segment in seconds.
        sail_angle_deg : float
            Sail angle in degrees (will be clamped to [-180, 180]).
        rudder_angle_deg : float
            Rudder angle in degrees (will be clamped to [-30, 30]).
        """
        self.start_time = start_time
        self.end_time = end_time
        self.sail_angle_rad = np.clip(np.deg2rad(sail_angle_deg), -np.pi, np.pi)
        self.rudder_angle_rad = np.clip(np.deg2rad(rudder_angle_deg), -np.pi/6, np.pi/6)

class saildrone:
    def __init__(self, M=2500, I=10000, rho=1.225, A_sail=15.0, course_params_list=[]):
        """
        Initialize the saildrone with physical parameters and course information.

        Parameters
        ----------
        M : float
            Mass of the saildrone in kg.
        I : float
            Moment of inertia of the saildrone in kg*m^2.
        rho : float
            Density of air in kg/m^3.
        A_sail : float
            Sail area in m^2.
        course_params_list : list of course_parameters
            List of course parameter objects defining the sail and rudder angles over time.
        """

        self.M = M  # Mass of the saildrone in kg
        self.I = I  # Moment of inertia of the saildrone in kg*m^2
        self.rho = rho  # Density of air in kg/m^3
        self.A_sail = A_sail  # Sail area in m^2
        self.course_params_list = course_params_list

    def get_active_course_params(self, t):
        """
        Find the course parameters that are active at time t.
        
        Parameters
        ----------
        t : float
            Current time in seconds.
            
        Returns
        -------
        course_parameters or None
            The active course parameters, or None if no interval matches.
        """
        for params in self.course_params_list:
            if params.start_time <= t <= params.end_time:
                return params
        return None
    
       
    def state_deriv_drone(self,t,s,sail_angle,rudder_angle):
        """
        Compute the derivative of the state vector for the saildrone.
        
        Parameters
        ----------
        t : float64
            Time in seconds.
        s : numpy.ndarray
            State vector with five elements: x-position in m, y-position in m,
            heading in radians, x-velocity in m/s, y-velocity in m/s and turn rate in rad/s.
        sail_angle : float
            Sail angle in radians.
        rudder_angle : float
            Rudder angle in radians.
        """

        M = 2500  # Mass of the saildrone in kg
        I = 10000  # Moment of inertia of the saildrone in kg*m^2
        rho = 1.225  # Density of air in kg/m^3
        A_sail = 15.0  # Sail area in m^2

        v_wind = np.array([-6.7, 0])

        wind_angle = np.arctan2(v_wind[1], v_wind[0])

        v_apparent_wind = v_wind - np.array([s[3], s[4]])
        aparent_wind_angle = np.arctan2(v_apparent_wind[1], v_apparent_wind[0])

        s_dot = np.zeros(6)
        s_dot[0] = s[3]                     # dx/dt = vx
        s_dot[1] = s[4]                     # dy/dt = vy
        s_dot[2] = s[5]                     # dtheta/dt = omega
        
        # s_dot[3], s_dot[4], s_dot[5] will be computed below

        force_hydro, torque_hydro = saildrone_hydro.get_vals(
            np.array([s[3], s[4]]), s[2], s[5], rudder_angle)

        attack_angle = -(s[2] + sail_angle) + aparent_wind_angle
        attack_angle = np.arctan2(np.sin(attack_angle), np.cos(attack_angle))
        
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
        s_dot[5] = 1/I * (torque_hydro + (-0.1*np.cos(s[2])*(F_d_y + F_l_y) + 0.1*np.sin(s[2])*(F_d_x + F_l_x)))

        return s_dot


    def runge_kutta4_step(self, t,dt,s,sail_angle,rudder_angle):
        """
        Apply the classical 4th-order Runge-Kutta method for one time step.

        Parameters
        ----------
        t : float64
            Time.
        dt : float64
            Time step.
        s : numpy.ndarray
            State vector at the current time step.
        sail_angle : float
            Sail angle in radians (held constant during this step).
        rudder_angle : float
            Rudder angle in radians (held constant during this step).
            
        Returns
        -------
        znext : numpy.ndarray
            State vector at the next time step.
        """
        
        A = self.state_deriv_drone(t, s, sail_angle, rudder_angle)
        B = self.state_deriv_drone(t + dt/2, s + dt/2 * A, sail_angle, rudder_angle)
        C = self.state_deriv_drone(t + dt/2, s + dt/2 * B, sail_angle, rudder_angle)
        D = self.state_deriv_drone(t + dt, s + dt * C, sail_angle, rudder_angle)

        snext = s + (dt/6) * (A + 2*B + 2*C + D)

        return snext



    def simulate_course(self, t0, tmax, dt, s0):
        """
        Simulate the saildrone trajectory with a sequence of course parameters.
        
        Parameters
        ----------
        course_params_list : list of course_parameters
            List of course parameter objects defining sail/rudder angles for different time periods.
        t0 : float
            Start time in seconds.
        tmax : float
            End time in seconds.
        dt : float
            Time step in seconds.
        s0 : numpy.ndarray
            Initial state vector [x, y, heading, vx, vy, turn_rate].
            
        Returns
        -------
        t : numpy.ndarray
            Time array.
        s : numpy.ndarray
            State array with shape (6, n_timesteps).
        """

        t = np.array([t0])
        s = s0
        n = 0

        while t[n] <= tmax:
            t = np.append(t, t[-1] + dt)
            params = self.get_active_course_params(t[n])
            sail_angle = params.sail_angle_rad
            rudder_angle = params.rudder_angle_rad
            snext = self.runge_kutta4_step(t[n], dt, s[:, n], sail_angle, rudder_angle)
            snext = snext[:, np.newaxis]
            s = np.append(s, snext, axis=1)
            n += 1

        # Plot trajectory with different colors for each course segment
        plt.figure(figsize=(12, 8))
        
        # Define colors for different segments (using muted colors)
        colors = plt.cm.jet(np.linspace(0, 1, len(self.course_params_list)))
        
        # Plot each segment with different color
        for i, params in enumerate(self.course_params_list):
            # Find indices corresponding to this time segment
            segment_mask = (t >= params.start_time) & (t <= params.end_time)
            segment_indices = np.where(segment_mask)[0]
            
            if len(segment_indices) > 0:
                label = f't={params.start_time:.0f}-{params.end_time:.0f}s (sail={np.rad2deg(params.sail_angle_rad):.1f}°, rudder={np.rad2deg(params.rudder_angle_rad):.1f}°)'
                plt.plot(s[0, segment_indices], s[1, segment_indices], 
                        color=colors[i], linewidth=2, label=label)
        
        # Mark start and end points
        plt.plot(s[0, 0], s[1, 0], 'go', markersize=12, label='Start', zorder=5)
        plt.plot(s[0, -1], s[1, -1], 'ro', markersize=12, label='End', zorder=5)
        
        plt.xlabel('X Position (m)', fontsize=12)
        plt.ylabel('Y Position (m)', fontsize=12)
        plt.title('Saildrone Trajectory', fontsize=14)
        plt.legend(fontsize=9, loc='best')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.show()

        return t, s
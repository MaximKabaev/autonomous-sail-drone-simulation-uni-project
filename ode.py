import numpy as np
import saildrone_hydro
import matplotlib.pyplot as plt


class AutonomousController:
    """
    Autonomous controller that navigates through waypoints using PID control.
    """
    def __init__(self, waypoints, wind=(-6.7, 0), Kp_rudder=0.5, lookahead_distance=5.0):
        """
        Initialize the autonomous controller.
        
        Parameters
        ----------
        waypoints : list of tuples or numpy arrays
            List of (x, y) waypoint coordinates in meters.
        wind : tuple or numpy array
            Wind velocity vector (vx, vy) in m/s. This is where the wind is BLOWING TO.
            Example: (6.7, 0) means wind blowing east at 6.7 m/s (wind FROM west).
        Kp_rudder : float
            Proportional gain for rudder control (typical range: 0.3-0.8).
        lookahead_distance : float
            Distance threshold to consider a waypoint reached (meters).
        """
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.wind = wind
        self.wind_direction = np.arctan2(wind[1], wind[0]) + np.pi
        self.wind_direction = np.arctan2(np.sin(self.wind_direction), np.cos(self.wind_direction))
        self.Kp_rudder = Kp_rudder
        self.lookahead_distance = lookahead_distance
        self.waypoint_reach_times = []
        
    def get_control_inputs(self, state, current_time=None):
        """
        Calculate rudder and sail angles based on current state and target waypoint.
        
        Parameters
        ----------
        state : numpy.ndarray
            State vector [x, y, theta, vx, vy, omega].
        current_time : float, optional
            Current simulation time for tracking waypoint reach times.
        
        Returns
        -------
        rudder_angle : float
            Rudder angle in radians.
        sail_angle : float
            Sail angle in radians.
        """
        current_pos = state[:2]
        current_heading = state[2]
        
        target_wp = self.waypoints[self.current_waypoint_idx]
        
        distance_to_waypoint = np.linalg.norm(current_pos - target_wp)
        
        if distance_to_waypoint < self.lookahead_distance:
            # Move to next waypoint
            if current_time is not None:
                self.waypoint_reach_times.append(current_time)
            self.current_waypoint_idx = min(
                self.current_waypoint_idx + 1, 
                len(self.waypoints) - 1
            )
            target_wp = self.waypoints[self.current_waypoint_idx]
        
        target_heading = np.arctan2(
            target_wp[1] - current_pos[1],
            target_wp[0] - current_pos[0]
        )
        
        heading_error = current_heading - target_heading

        # normilize heading error angle
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        deadband = np.deg2rad(2)
        if abs(heading_error) < deadband:
            heading_error_compensated = 0
        else:
            #times by deadband for a smoother rotation of rudder
            heading_error_compensated = heading_error - np.sign(heading_error) * deadband
        
        # steer towards the waypoint
        rudder_desired = self.Kp_rudder * heading_error_compensated
        rudder_angle = np.clip(rudder_desired, -np.deg2rad(30), np.deg2rad(30))
        
        apparent_wind = self.wind_direction - current_heading
        apparent_wind = np.arctan2(np.sin(apparent_wind), np.cos(apparent_wind))
        
        # Calculate desired sail angle before clipping
        sail_desired = apparent_wind / 2
        sail_angle = np.clip(sail_desired, -np.pi, np.pi)
        
        # Store control data for plotting
        if not hasattr(self, 'control_history'):
            self.control_history = {
                'heading_error': [],
                'rudder_desired': [],
                'rudder_actual': [],
                'sail_desired': [],
                'sail_actual': []
            }
        
        self.control_history['heading_error'].append(heading_error)
        self.control_history['rudder_desired'].append(rudder_desired)
        self.control_history['rudder_actual'].append(rudder_angle)
        self.control_history['sail_desired'].append(sail_desired)
        self.control_history['sail_actual'].append(sail_angle)
        
        return rudder_angle, sail_angle
    
    def reset(self):
        """Reset controller to start from first waypoint."""
        self.current_waypoint_idx = 0
    
    def all_waypoints_reached(self):
        """Check if all waypoints have been reached."""
        return len(self.waypoint_reach_times) >= len(self.waypoints)

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
        self.rudder_angle_rad = np.clip(np.deg2rad(rudder_angle_deg), -np.deg2rad(30), np.deg2rad(30))

class saildrone:
    def __init__(self, M=2500, I=10000, rho=1.225, A_sail=15.0, course_params_list=[], controller=None, wind_speed=6.7):
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
            (Used for manual control mode - ignored if controller is provided)
        controller : AutonomousController, optional
            Autonomous controller for waypoint navigation. If provided, overrides course_params_list.
        wind_speed : float
            Wind speed in m/s (default 6.7 m/s).
        """

        self.M = M  # Mass of the saildrone in kg
        self.I = I  # Moment of inertia of the saildrone in kg*m^2
        self.rho = rho  # Density of air in kg/m^3
        self.A_sail = A_sail  # Sail area in m^2
        self.course_params_list = course_params_list
        self.controller = controller  # Autonomous controller
        self.wind_speed = wind_speed  # Wind speed in m/s
        
        # Initialize force history tracking
        self.force_history = {
            'drag_x': [],
            'drag_y': [],
            'lift_x': [],
            'lift_y': [],
            'hydro_x': [],
            'hydro_y': [],
            'torque_hydro': [],
            'torque_aero': [],
            'apparent_wind_angle': [],
            'heading': [],
            'attack_angle': []
        }

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

        # Get wind velocity from controller if available, otherwise use default
        if self.controller is not None:
            v_wind = self.controller.wind
        else:
            v_wind = np.array([-self.wind_speed, 0])  # Default: wind blowing west (from east)

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

        attack_angle = (s[2] - sail_angle) + aparent_wind_angle
        attack_angle = np.arctan2(np.sin(attack_angle), np.cos(attack_angle))
        
        c_d = 1 - np.cos(2*(attack_angle))
        c_l = 1.5*np.sin(2*(attack_angle) + 0.5 * np.sin(2*(attack_angle)))
        F_d = 0.5 * rho * A_sail * np.linalg.norm(v_apparent_wind)**2 * c_d
        F_l = 0.5 * rho * A_sail * np.linalg.norm(v_apparent_wind)**2 * c_l
        F_d_x = F_d * -np.cos(aparent_wind_angle)
        F_d_y = F_d * np.sin(aparent_wind_angle)
        F_l_x = F_l * np.sin(aparent_wind_angle)
        F_l_y = F_l * np.cos(aparent_wind_angle)
        
        torque_aero = (-0.1*np.cos(s[2])*(F_d_y + F_l_y) + 0.1*np.sin(s[2])*(F_d_x + F_l_x))
        
        s_dot[3] = 1/M * (F_d_x + F_l_x + force_hydro[0])
        s_dot[4] = 1/M * (F_d_y + F_l_y + force_hydro[1])
        s_dot[5] = 1/I * (torque_hydro + torque_aero)
        
        # Store force components for plotting
        self.force_history['drag_x'].append(F_d_x)
        self.force_history['drag_y'].append(F_d_y)
        self.force_history['lift_x'].append(F_l_x)
        self.force_history['lift_y'].append(F_l_y)
        self.force_history['hydro_x'].append(force_hydro[0])
        self.force_history['hydro_y'].append(force_hydro[1])
        self.force_history['torque_hydro'].append(torque_hydro)
        self.force_history['torque_aero'].append(torque_aero)
        self.force_history['apparent_wind_angle'].append(aparent_wind_angle)
        self.force_history['heading'].append(s[2])
        self.force_history['attack_angle'].append(attack_angle)

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


    def simulate_course(self, t0, dt, s0, plot=True):
        """
        Simulate the saildrone trajectory with a sequence of course parameters.
        
        Parameters
        ----------
        t0 : float
            Start time in seconds.
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
        # Reset force history for new simulation
        self.force_history = {
            'drag_x': [],
            'drag_y': [],
            'lift_x': [],
            'lift_y': [],
            'hydro_x': [],
            'hydro_y': [],
            'torque_hydro': [],
            'torque_aero': [],
            'apparent_wind_angle': [],
            'heading': [],
            'attack_angle': []
        }
        
        t = np.array([t0])
        s = s0
        n = 0

        while t[n] < 3500:
            t = np.append(t, t[-1] + dt)
            
            # Use autonomous controller if available, otherwise use course_params_list
            if self.controller is not None:
                rudder_angle, sail_angle = self.controller.get_control_inputs(s[:, n], t[n])
                if self.controller.all_waypoints_reached():
                    break
            else:
                params = self.get_active_course_params(t[n])
                if params is None:
                    break
                sail_angle = params.sail_angle_rad
                rudder_angle = params.rudder_angle_rad
            
            snext = self.runge_kutta4_step(t[n], dt, s[:, n], sail_angle, rudder_angle)
            snext = snext[:, np.newaxis]
            s = np.append(s, snext, axis=1)
            n += 1

        if plot:
            # Plot trajectory
            plt.figure(figsize=(12, 8))
            
            # Plot based on control mode
            if self.controller is not None:
                # Autonomous mode: plot trajectory and waypoints
                plt.plot(s[0, :], s[1, :], 'b-', linewidth=2, label='Drone Path')
                
                # Plot waypoints
                waypoints = np.array(self.controller.waypoints)
                plt.plot(waypoints[:, 0], waypoints[:, 1], 'r*', markersize=15, 
                        label='Waypoints', zorder=5)
                
                # Connect waypoints with dashed lines
                plt.plot(waypoints[:, 0], waypoints[:, 1], 'r--', alpha=0.3, linewidth=1)
                
                # Number the waypoints and show reach times
                for i, wp in enumerate(waypoints):
                    if i < len(self.controller.waypoint_reach_times):
                        reach_time = self.controller.waypoint_reach_times[i]
                        label = f'WP{i+1}\nt={reach_time:.1f}s'
                    else:
                        label = f'WP{i+1}'
                    plt.annotate(label, xy=(wp[0], wp[1]), 
                            xytext=(5, 5), textcoords='offset points',
                            fontsize=10, color='red')
            else:
                # Manual mode: plot with different colors for each course segment
                colors = plt.cm.jet(np.linspace(0, 1, len(self.course_params_list)))
                
                for i, params in enumerate(self.course_params_list):
                    segment_mask = (t >= params.start_time) & (t <= params.end_time)
                    segment_indices = np.where(segment_mask)[0]
                    
                    if len(segment_indices) > 0:
                        label = f't={params.start_time:.0f}-{params.end_time:.0f}s (sail={np.rad2deg(params.sail_angle_rad):.1f}°, rudder={np.rad2deg(params.rudder_angle_rad):.1f}°)'
                        plt.plot(s[0, segment_indices], s[1, segment_indices], 
                                color=colors[i], linewidth=2, label=label)
            
            # Mark start and end points
            plt.plot(s[0, 0], s[1, 0], 'go', markersize=12, label='Start', zorder=5)
            plt.plot(s[0, -1], s[1, -1], 'ro', markersize=12, label='End', zorder=5)
            
            # Plot wind direction arrow
            if self.controller is not None:
                wind_vector = self.controller.wind
            else:
                wind_vector = np.array([-self.wind_speed, 0])  # Default wind blowing west
            
            # Position arrow in bottom-right corner of the entire plot axes
            ax = plt.gca()
            x_lim = ax.get_xlim()
            y_lim = ax.get_ylim()
            x_range = x_lim[1] - x_lim[0]
            y_range = y_lim[1] - y_lim[0]
            
            arrow_x = x_lim[1] + 3 * x_range
            arrow_y = y_lim[0] + 0.15 * y_range
            arrow_length = 0.1 * max(x_range, y_range)
            
            # Normalize wind vector and scale to arrow length
            wind_magnitude = np.linalg.norm(wind_vector)
            if wind_magnitude > 0:
                wind_dx = arrow_length * wind_vector[0] / wind_magnitude
                wind_dy = arrow_length * wind_vector[1] / wind_magnitude
            else:
                wind_dx, wind_dy = 0, 0
            
            # Draw arrow showing where wind is BLOWING TO
            plt.arrow(arrow_x, arrow_y, wind_dx, wind_dy, 
                    head_width=arrow_length*0.3, head_length=arrow_length*0.25,
                    fc='cyan', ec='blue', linewidth=2, label='Wind Direction', zorder=10)
            
            # Add wind vector text
            plt.text(arrow_x, arrow_y - 0.05 * y_range, 
                    f'Wind: ({wind_vector[0]:.1f}, {wind_vector[1]:.1f}) m/s',
                    fontsize=10, color='blue', ha='center', weight='bold')
            
            plt.xlabel('X Position (m)', fontsize=12)
            plt.ylabel('Y Position (m)', fontsize=12)
            plt.title('Saildrone Trajectory', fontsize=14)
            plt.legend(fontsize=9, loc='best')
            plt.grid(True, alpha=0.3)
            plt.axis('equal')

            # Plot 2: Heading Error Over Time
            if self.controller and hasattr(self.controller, 'control_history'):
                history = self.controller.control_history
                actual_tmax = t[-1]
                time_steps = np.linspace(0, actual_tmax, len(history['heading_error']))
                
                plt.figure(figsize=(10, 5))
                plt.plot(time_steps, np.rad2deg(history['heading_error']), 'b-', linewidth=2)
                
                # Plot waypoint reach times
                for i, reach_time in enumerate(self.controller.waypoint_reach_times):
                    plt.axvline(x=reach_time, color='red', linestyle='--', alpha=0.5, linewidth=1.5)
                    plt.text(reach_time, plt.ylim()[1]*0.9, f'WP{i+1}', color='red', fontsize=9, ha='center')
                
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Heading Error (degrees)', fontsize=12)
                plt.title('Heading Error Over Time', fontsize=14)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)

                # Plot 3: Control Angles Over Time
                plt.figure(figsize=(10, 8))
                
                # Rudder angles subplot
                plt.subplot(2, 1, 1)
                plt.plot(time_steps, np.rad2deg(history['rudder_desired']), 'r--', 
                        linewidth=2, label='Desired Rudder Angle', alpha=0.7)
                plt.plot(time_steps, np.rad2deg(history['rudder_actual']), 'r-', 
                        linewidth=2, label='Actual Rudder Angle')
                plt.axhline(y=30, color='k', linestyle=':', alpha=0.5, label='Limit (±30°)')
                plt.axhline(y=-30, color='k', linestyle=':', alpha=0.5)
                
                # Plot waypoint reach times
                for i, reach_time in enumerate(self.controller.waypoint_reach_times):
                    plt.axvline(x=reach_time, color='red', linestyle='--', alpha=0.5, linewidth=1.5)
                    plt.text(reach_time, plt.ylim()[1]*0.9, f'WP{i+1}', color='red', fontsize=9, ha='center')
                
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Rudder Angle (degrees)', fontsize=12)
                plt.title('Rudder Angle Over Time', fontsize=14)
                plt.legend(fontsize=10)
                plt.grid(True, alpha=0.3)
                
                # Sail angles subplot
                plt.subplot(2, 1, 2)
                plt.plot(time_steps, np.rad2deg(history['sail_desired']), 'g--', 
                        linewidth=2, label='Desired Sail Angle', alpha=0.7)
                plt.plot(time_steps, np.rad2deg(history['sail_actual']), 'g-', 
                        linewidth=2, label='Actual Sail Angle')
                plt.axhline(y=180, color='k', linestyle=':', alpha=0.5, label='Limit (±180°)')
                plt.axhline(y=-180, color='k', linestyle=':', alpha=0.5)
                
                # Plot waypoint reach times
                for i, reach_time in enumerate(self.controller.waypoint_reach_times):
                    plt.axvline(x=reach_time, color='red', linestyle='--', alpha=0.5, linewidth=1.5)
                    plt.text(reach_time, plt.ylim()[1]*0.9, f'WP{i+1}', color='red', fontsize=9, ha='center')
                
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Sail Angle (degrees)', fontsize=12)
                plt.title('Sail Angle Over Time', fontsize=14)
                plt.legend(fontsize=10)
                plt.grid(True, alpha=0.3)
                
                plt.tight_layout()

            # Plot 4: Velocity Over Time
            plt.figure(figsize=(10, 10))
            
            # Ensure t and s have matching dimensions
            min_len = min(len(t), s.shape[1])
            t_plot = t[:min_len]
            
            # Calculate overall velocity magnitude
            velocity_magnitude = np.sqrt(s[3, :min_len]**2 + s[4, :min_len]**2)
            
            # Overall Velocity subplot
            plt.subplot(3, 1, 1)
            plt.plot(t_plot, velocity_magnitude, 'g-', linewidth=2, label='Velocity')
            plt.xlabel('Time (s)', fontsize=12)
            plt.ylabel('Velocity (m/s)', fontsize=12)
            plt.title('Velocity Over Time', fontsize=14)
            plt.legend(fontsize=10)
            plt.grid(True, alpha=0.3)
            
            # X Velocity subplot
            plt.subplot(3, 1, 2)
            plt.plot(t_plot, s[3, :min_len], 'b-', linewidth=2, label='X Velocity')
            plt.xlabel('Time (s)', fontsize=12)
            plt.ylabel('X Velocity (m/s)', fontsize=12)
            plt.title('X Velocity Over Time', fontsize=14)
            plt.legend(fontsize=10)
            plt.grid(True, alpha=0.3)
            plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            
            # Y Velocity subplot
            plt.subplot(3, 1, 3)
            plt.plot(t_plot, s[4, :min_len], 'r-', linewidth=2, label='Y Velocity')
            plt.xlabel('Time (s)', fontsize=12)
            plt.ylabel('Y Velocity (m/s)', fontsize=12)
            plt.title('Y Velocity Over Time', fontsize=14)
            plt.legend(fontsize=10)
            plt.grid(True, alpha=0.3)
            plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            
            plt.tight_layout()

            # Plot 5: Force Components Over Time
            if len(self.force_history['drag_x']) > 0:
                plt.figure(figsize=(12, 12))
                
                # Ensure force history has matching dimensions with time
                force_len = len(self.force_history['drag_x'])
                # Since state_deriv_drone is called multiple times per timestep (RK4),
                # we need to subsample to match the simulation timesteps
                # Take every 4th sample (k1, k2, k3, k4 in RK4)
                step = 4
                indices = np.arange(0, force_len, step)[:min_len]
                t_force = t_plot[:len(indices)]
                
                # Drag Force Components
                plt.subplot(4, 2, 1)
                plt.plot(t_force, np.array(self.force_history['drag_x'])[indices], 'b-', linewidth=2)
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Drag Force X (N)', fontsize=12)
                plt.title('Drag Force X Component Over Time', fontsize=14)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                plt.subplot(4, 2, 2)
                plt.plot(t_force, np.array(self.force_history['drag_y'])[indices], 'r-', linewidth=2)
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Drag Force Y (N)', fontsize=12)
                plt.title('Drag Force Y Component Over Time', fontsize=14)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                # Lift Force Components
                plt.subplot(4, 2, 3)
                plt.plot(t_force, np.array(self.force_history['lift_x'])[indices], 'b-', linewidth=2)
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Lift Force X (N)', fontsize=12)
                plt.title('Lift Force X Component Over Time', fontsize=14)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                plt.subplot(4, 2, 4)
                plt.plot(t_force, np.array(self.force_history['lift_y'])[indices], 'r-', linewidth=2)
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Lift Force Y (N)', fontsize=12)
                plt.title('Lift Force Y Component Over Time', fontsize=14)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                # Hydrodynamic Force Components
                plt.subplot(4, 2, 5)
                plt.plot(t_force, np.array(self.force_history['hydro_x'])[indices], 'b-', linewidth=2)
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Hydro Force X (N)', fontsize=12)
                plt.title('Hydrodynamic Force X Component Over Time', fontsize=14)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                plt.subplot(4, 2, 6)
                plt.plot(t_force, np.array(self.force_history['hydro_y'])[indices], 'r-', linewidth=2)
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Hydro Force Y (N)', fontsize=12)
                plt.title('Hydrodynamic Force Y Component Over Time', fontsize=14)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                # Torque Components
                plt.subplot(4, 2, 7)
                plt.plot(t_force, np.array(self.force_history['torque_hydro'])[indices], 'g-', linewidth=2)
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Hydro Torque (N·m)', fontsize=12)
                plt.title('Hydrodynamic Torque Over Time', fontsize=14)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                plt.subplot(4, 2, 8)
                plt.plot(t_force, np.array(self.force_history['torque_aero'])[indices], 'm-', linewidth=2)
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Aero Torque (N·m)', fontsize=12)
                plt.title('Aerodynamic Torque Over Time', fontsize=14)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                plt.tight_layout()

            # Plot 6: Overall Force Magnitudes Over Time
            if len(self.force_history['drag_x']) > 0:
                plt.figure(figsize=(10, 10))
                
                # Calculate overall force magnitudes
                drag_magnitude = np.sqrt(
                    np.array(self.force_history['drag_x'])**2 + 
                    np.array(self.force_history['drag_y'])**2
                )[indices]
                
                lift_magnitude = np.sqrt(
                    np.array(self.force_history['lift_x'])**2 + 
                    np.array(self.force_history['lift_y'])**2
                )[indices]
                
                hydro_magnitude = np.sqrt(
                    np.array(self.force_history['hydro_x'])**2 + 
                    np.array(self.force_history['hydro_y'])**2
                )[indices]
                
                # Overall Drag Force
                plt.subplot(3, 1, 1)
                plt.plot(t_force, drag_magnitude, 'b-', linewidth=2, label='Drag Force')
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Drag Force (N)', fontsize=12)
                plt.title('Overall Drag Force Magnitude Over Time', fontsize=14)
                plt.legend(fontsize=10)
                plt.grid(True, alpha=0.3)
                
                # Overall Lift Force
                plt.subplot(3, 1, 2)
                plt.plot(t_force, lift_magnitude, 'g-', linewidth=2, label='Lift Force')
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Lift Force (N)', fontsize=12)
                plt.title('Overall Lift Force Magnitude Over Time', fontsize=14)
                plt.legend(fontsize=10)
                plt.grid(True, alpha=0.3)
                
                # Overall Hydrodynamic Force
                plt.subplot(3, 1, 3)
                plt.plot(t_force, hydro_magnitude, 'r-', linewidth=2, label='Hydrodynamic Force')
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Hydro Force (N)', fontsize=12)
                plt.title('Overall Hydrodynamic Force Magnitude Over Time', fontsize=14)
                plt.legend(fontsize=10)
                plt.grid(True, alpha=0.3)
                
                plt.tight_layout()

            # Plot 7: Total Force Components (Sum of all forces in X and Y)
            if len(self.force_history['drag_x']) > 0:
                plt.figure(figsize=(10, 6))
                
                # Calculate total force in X and Y directions
                total_force_x = (
                    np.array(self.force_history['drag_x']) + 
                    np.array(self.force_history['lift_x']) + 
                    np.array(self.force_history['hydro_x'])
                )[indices]
                
                total_force_y = (
                    np.array(self.force_history['drag_y']) + 
                    np.array(self.force_history['lift_y']) + 
                    np.array(self.force_history['hydro_y'])
                )[indices]
                
                # Total X Force
                plt.subplot(2, 1, 1)
                plt.plot(t_force, total_force_x, 'b-', linewidth=2, label='Total X Force')
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Total Force X (N)', fontsize=12)
                plt.title('Total Force X Component Over Time', fontsize=14)
                plt.legend(fontsize=10)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                # Total Y Force
                plt.subplot(2, 1, 2)
                plt.plot(t_force, total_force_y, 'r-', linewidth=2, label='Total Y Force')
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Total Force Y (N)', fontsize=12)
                plt.title('Total Force Y Component Over Time', fontsize=14)
                plt.legend(fontsize=10)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                plt.tight_layout()

            # Plot 8: Apparent Wind Angle, Heading, and Attack Angle Over Time
            if len(self.force_history['apparent_wind_angle']) > 0:
                plt.figure(figsize=(10, 9))
                
                # Apparent Wind Angle
                plt.subplot(3, 1, 1)
                plt.plot(t_force, np.rad2deg(np.array(self.force_history['apparent_wind_angle'])[indices]), 'c-', linewidth=2, label='Apparent Wind Angle')
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Apparent Wind Angle (degrees)', fontsize=12)
                plt.title('Apparent Wind Angle Over Time', fontsize=14)
                plt.legend(fontsize=10)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                # Heading
                plt.subplot(3, 1, 2)
                plt.plot(t_force, np.rad2deg(np.array(self.force_history['heading'])[indices]), 'm-', linewidth=2, label='Heading')
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Heading (degrees)', fontsize=12)
                plt.title('Heading Over Time', fontsize=14)
                plt.legend(fontsize=10)
                plt.grid(True, alpha=0.3)
                
                # Attack Angle
                plt.subplot(3, 1, 3)
                plt.plot(t_force, np.rad2deg(np.array(self.force_history['attack_angle'])[indices]), 'r-', linewidth=2, label='Attack Angle')
                plt.axvline(x=65, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                plt.xlabel('Time (s)', fontsize=12)
                plt.ylabel('Attack Angle (degrees)', fontsize=12)
                plt.title('Attack Angle Over Time', fontsize=14)
                plt.legend(fontsize=10)
                plt.grid(True, alpha=0.3)
                plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                plt.tight_layout()

            # Show all plots at once
            plt.show()

        return t, s
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
import ode


class SailDroneGUI:
    def __init__(self):
        # Initial state vector (hardcoded)
        self.s0 = np.array([
            [0.0],
            [0.0],
            [np.pi/2],
            [0.0],
            [2.9],
            [0.0]
        ])

        # Wind vector (hardcoded) - where wind blows TO
        self.wind = np.array([-6.7, 0])

        # Waypoints list
        self.waypoints = []

        # Controller parameters
        self.kp_rudder = 0.5
        self.lookahead_distance = 1.0

        # Axis limits for zoom
        self.x_min = -100
        self.x_max = 500
        self.y_min = -100
        self.y_max = 500

        # Panning state
        self.is_panning = False
        self.pan_start_x = None
        self.pan_start_y = None
        self.pan_start_xlim = None
        self.pan_start_ylim = None

        # Create GUI
        self.create_gui()

    def create_gui(self):
        """Create the interactive GUI using matplotlib."""
        self.fig = plt.figure(figsize=(14, 9))

        # Main plot area
        self.ax = plt.axes([0.08, 0.25, 0.65, 0.65])

        # Set up plot limits
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.ax.set_title('Sail Drone Simulation - Click to Add Waypoints', fontsize=14)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')

        # Draw initial state
        self.draw_initial_state()

        # Right panel for state configuration
        panel_x = 0.76
        row_height = 0.035
        row_start = 0.83

        # Header
        self.fig.text(panel_x + 0.05, row_start + 0.055, 'Initial State Configuration',
                     fontsize=11, weight='bold')

        # Initial Position X
        ax_pos_x = plt.axes([panel_x, row_start, 0.18, 0.03])
        self.text_pos_x = TextBox(ax_pos_x, 'Init X (m):', initial=f'{self.s0[0,0]:.1f}')
        self.text_pos_x.on_submit(self.update_state)

        # Initial Position Y
        ax_pos_y = plt.axes([panel_x, row_start - row_height, 0.18, 0.03])
        self.text_pos_y = TextBox(ax_pos_y, 'Init Y (m):', initial=f'{self.s0[1,0]:.1f}')
        self.text_pos_y.on_submit(self.update_state)

        # Initial Heading (in degrees)
        ax_heading = plt.axes([panel_x, row_start - 2*row_height, 0.18, 0.03])
        self.text_heading = TextBox(ax_heading, 'Heading (°):', initial=f'{np.rad2deg(self.s0[2,0]):.1f}')
        self.text_heading.on_submit(self.update_state)

        # Initial Velocity X
        ax_vel_x = plt.axes([panel_x, row_start - 3*row_height, 0.18, 0.03])
        self.text_vel_x = TextBox(ax_vel_x, 'Vel X (m/s):', initial=f'{self.s0[3,0]:.1f}')
        self.text_vel_x.on_submit(self.update_state)

        # Initial Velocity Y
        ax_vel_y = plt.axes([panel_x, row_start - 4*row_height, 0.18, 0.03])
        self.text_vel_y = TextBox(ax_vel_y, 'Vel Y (m/s):', initial=f'{self.s0[4,0]:.1f}')
        self.text_vel_y.on_submit(self.update_state)

        # Wind configuration header
        self.fig.text(panel_x + 0.05, row_start - 5.5*row_height, 'Wind Configuration',
                     fontsize=11, weight='bold')

        # Wind X
        ax_wind_x = plt.axes([panel_x, row_start - 6.5*row_height, 0.18, 0.03])
        self.text_wind_x = TextBox(ax_wind_x, 'Wind X (m/s):', initial=f'{self.wind[0]:.1f}')
        self.text_wind_x.on_submit(self.update_wind)

        # Wind Y
        ax_wind_y = plt.axes([panel_x, row_start - 7.5*row_height, 0.18, 0.03])
        self.text_wind_y = TextBox(ax_wind_y, 'Wind Y (m/s):', initial=f'{self.wind[1]:.1f}')
        self.text_wind_y.on_submit(self.update_wind)

        # Waypoint counter
        self.waypoint_text = self.fig.text(0.08, 0.20, 'Waypoints: 0', fontsize=11, weight='bold')

        # Controller parameters header
        self.fig.text(panel_x + 0.05, row_start - 9*row_height, 'Controller Parameters',
                     fontsize=11, weight='bold')

        # Text input: Kp
        ax_kp = plt.axes([panel_x, row_start - 10*row_height, 0.18, 0.03])
        self.text_kp = TextBox(ax_kp, 'Rudder Kp:', initial=f'{self.kp_rudder:.1f}')
        self.text_kp.on_submit(self.update_kp)

        # Text input: Lookahead
        ax_lookahead = plt.axes([panel_x, row_start - 11*row_height, 0.18, 0.03])
        self.text_lookahead = TextBox(ax_lookahead, 'Lookahead (m):', initial=f'{self.lookahead_distance:.1f}')
        self.text_lookahead.on_submit(self.update_lookahead)

        # Button: Clear Waypoints
        ax_clear = plt.axes([0.25, 0.15, 0.15, 0.04])
        self.btn_clear = Button(ax_clear, 'Clear Waypoints')
        self.btn_clear.on_clicked(self.clear_waypoints)

        # Button: Run Simulation
        ax_run = plt.axes([0.45, 0.15, 0.15, 0.04])
        self.btn_run = Button(ax_run, 'Run Simulation')
        self.btn_run.on_clicked(self.run_simulation)

        # Instructions
        self.fig.text(0.4, 0.01, 'Left-click: add waypoint | Right-click & drag: pan | Scroll: zoom | Edit params on right (Enter to apply)',
                     ha='center', fontsize=9, color='blue', style='italic')

        # Connect mouse events
        self.cid_press = self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_release)
        self.cid_motion = self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_motion)
        self.sid = self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)

        plt.show()

    def draw_initial_state(self):
        """Draw the boat and wind on the canvas."""
        # Draw boat initial position and orientation
        boat_x, boat_y = self.s0[0, 0], self.s0[1, 0]
        boat_heading = self.s0[2, 0]

        # Draw boat as a triangle pointing in the direction of heading
        arrow_length = 30
        dx = arrow_length * np.cos(boat_heading)
        dy = arrow_length * np.sin(boat_heading)

        self.ax.arrow(boat_x, boat_y, dx, dy,
                     head_width=15, head_length=20,
                     fc='green', ec='darkgreen', linewidth=2,
                     label='Boat Start', zorder=10)

        # Draw wind direction
        wind_magnitude = np.linalg.norm(self.wind)
        if wind_magnitude > 0:
            x_min, x_max = self.ax.get_xlim()
            y_min, y_max = self.ax.get_ylim()
            x_range = x_max - x_min
            y_range = y_max - y_min

            wind_x = x_max - 0.15 * x_range
            wind_y = y_min + 0.15 * y_range
            wind_arrow_length = 0.08 * max(x_range, y_range)

            wind_dx = wind_arrow_length * self.wind[0] / wind_magnitude
            wind_dy = wind_arrow_length * self.wind[1] / wind_magnitude

            self.ax.arrow(wind_x, wind_y, wind_dx, wind_dy,
                         head_width=wind_arrow_length*0.3,
                         head_length=wind_arrow_length*0.25,
                         fc='cyan', ec='blue', linewidth=3,
                         label='Wind Direction', zorder=10)

            self.ax.text(wind_x, wind_y - 0.05 * y_range,
                        f'Wind: ({self.wind[0]:.1f}, {self.wind[1]:.1f}) m/s',
                        fontsize=11, color='blue', ha='center', weight='bold')

        self.ax.legend(fontsize=10, loc='upper left')

    def update_canvas(self):
        """Update the canvas with current waypoints."""
        # Clear only waypoint-related plots
        self.ax.clear()

        # Keep axis limits fixed BEFORE redrawing
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)

        # Redraw initial state
        self.draw_initial_state()

        # Draw waypoints
        if len(self.waypoints) > 0:
            waypoints_array = np.array(self.waypoints)
            self.ax.plot(waypoints_array[:, 0], waypoints_array[:, 1],
                        'r*', markersize=15, label='Waypoints', zorder=5)

            # Connect waypoints with dashed lines
            self.ax.plot(waypoints_array[:, 0], waypoints_array[:, 1],
                        'r--', alpha=0.3, linewidth=1)

            # Number the waypoints
            for i, (x, y) in enumerate(self.waypoints):
                self.ax.annotate(f'WP{i+1}', xy=(x, y),
                               xytext=(5, 5), textcoords='offset points',
                               fontsize=10, color='red', weight='bold')

        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.ax.set_title('Sail Drone Simulation - Click to Add Waypoints', fontsize=14)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')

        self.fig.canvas.draw()

    def on_mouse_press(self, event):
        """Handle mouse button press."""
        if event.inaxes != self.ax:
            return

        if event.xdata is None or event.ydata is None:
            return

        # Left click: add waypoint
        if event.button == 1:  # Left mouse button
            self.waypoints.append((event.xdata, event.ydata))
            self.waypoint_text.set_text(f'Waypoints: {len(self.waypoints)}')
            self.update_canvas()

        # Right click or middle click: start panning
        elif event.button in [2, 3]:  # Middle or right mouse button
            self.is_panning = True
            self.pan_start_x = event.xdata
            self.pan_start_y = event.ydata
            self.pan_start_xlim = (self.x_min, self.x_max)
            self.pan_start_ylim = (self.y_min, self.y_max)
            # Change cursor to indicate panning mode
            try:
                self.fig.canvas.toolbar.set_cursor(3)  # Hand cursor
            except:
                pass

    def on_mouse_release(self, event):
        """Handle mouse button release."""
        if event.button in [2, 3]:  # Middle or right mouse button
            self.is_panning = False
            self.pan_start_x = None
            self.pan_start_y = None
            # Reset cursor
            try:
                self.fig.canvas.toolbar.set_cursor(1)  # Default cursor
            except:
                pass

    def on_mouse_motion(self, event):
        """Handle mouse motion for panning."""
        if not self.is_panning:
            return

        if event.inaxes != self.ax:
            return

        if event.xdata is None or event.ydata is None:
            return

        # Calculate how much to pan
        dx = self.pan_start_x - event.xdata
        dy = self.pan_start_y - event.ydata

        # Update axis limits
        self.x_min = self.pan_start_xlim[0] + dx
        self.x_max = self.pan_start_xlim[1] + dx
        self.y_min = self.pan_start_ylim[0] + dy
        self.y_max = self.pan_start_ylim[1] + dy

        # Redraw canvas
        self.update_canvas()

    def on_scroll(self, event):
        """Handle scroll event for zooming."""
        if event.inaxes != self.ax:
            return

        # Zoom factor
        zoom_factor = 1.2

        # Get current mouse position
        x_mouse = event.xdata
        y_mouse = event.ydata

        if x_mouse is None or y_mouse is None:
            return

        # Calculate current ranges
        x_range = self.x_max - self.x_min
        y_range = self.y_max - self.y_min

        # Zoom in or out
        if event.button == 'up':  # Scroll up = zoom in
            new_x_range = x_range / zoom_factor
            new_y_range = y_range / zoom_factor
        elif event.button == 'down':  # Scroll down = zoom out
            new_x_range = x_range * zoom_factor
            new_y_range = y_range * zoom_factor
        else:
            return

        # Calculate how much the mouse is from the edges (as a ratio)
        x_ratio = (x_mouse - self.x_min) / x_range
        y_ratio = (y_mouse - self.y_min) / y_range

        # Update axis limits to zoom around the mouse position
        self.x_min = x_mouse - new_x_range * x_ratio
        self.x_max = x_mouse + new_x_range * (1 - x_ratio)
        self.y_min = y_mouse - new_y_range * y_ratio
        self.y_max = y_mouse + new_y_range * (1 - y_ratio)

        # Redraw canvas
        self.update_canvas()

    def clear_waypoints(self, event):
        """Clear all waypoints."""
        self.waypoints = []
        self.waypoint_text.set_text('Waypoints: 0')
        self.update_canvas()

    def update_kp(self, text):
        """Update Kp parameter."""
        try:
            self.kp_rudder = float(text)
            print(f"Rudder Kp updated to: {self.kp_rudder}")
        except ValueError:
            print("Invalid Kp value")

    def update_lookahead(self, text):
        """Update lookahead parameter."""
        try:
            self.lookahead_distance = float(text)
            print(f"Lookahead distance updated to: {self.lookahead_distance}")
        except ValueError:
            print("Invalid lookahead value")

    def update_state(self, text):
        """Update initial state vector from text inputs."""
        try:
            pos_x = float(self.text_pos_x.text)
            pos_y = float(self.text_pos_y.text)
            heading_deg = float(self.text_heading.text)
            vel_x = float(self.text_vel_x.text)
            vel_y = float(self.text_vel_y.text)

            # Update state vector
            self.s0 = np.array([
                [pos_x],
                [pos_y],
                [np.deg2rad(heading_deg)],
                [vel_x],
                [vel_y],
                [0.0]  # angular velocity stays 0
            ])

            print(f"Initial state updated: pos=({pos_x}, {pos_y}), heading={heading_deg}°, vel=({vel_x}, {vel_y})")
            self.update_canvas()
        except ValueError:
            print("Invalid state values")

    def update_wind(self, text):
        """Update wind vector from text inputs."""
        try:
            wind_x = float(self.text_wind_x.text)
            wind_y = float(self.text_wind_y.text)

            # Update wind vector
            self.wind = np.array([wind_x, wind_y])

            print(f"Wind updated to: ({wind_x}, {wind_y}) m/s")
            self.update_canvas()
        except ValueError:
            print("Invalid wind values")

    def run_simulation(self, event):
        """Run the simulation with current waypoints."""
        if len(self.waypoints) == 0:
            print("Please add at least one waypoint before running the simulation.")
            return

        # Create controller
        controller = ode.AutonomousController(
            waypoints=self.waypoints,
            wind=tuple(self.wind),
            Kp_rudder=self.kp_rudder,
            lookahead_distance=self.lookahead_distance
        )

        # Create saildrone
        saildrone = ode.saildrone(controller=controller)

        # Run simulation
        print("Running simulation...")
        print(f"Waypoints: {self.waypoints}")
        print(f"Kp_rudder: {self.kp_rudder}, Lookahead: {self.lookahead_distance}")
        saildrone.simulate_course(0, 0.1, self.s0)
        print("Simulation complete!")


def main():
    gui = SailDroneGUI()


if __name__ == '__main__':
    main()

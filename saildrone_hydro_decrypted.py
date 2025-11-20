import numpy as np


def calculate_lift_drag_force(coefficient, density, area, velocity):
    """
    Calculate hydrodynamic force using the standard drag/lift equation.

    Args:
        coefficient: Lift or drag coefficient (dimensionless)
        density: Fluid density (kg/m³)
        area: Surface area (m²)
        velocity: Flow velocity (m/s)

    Returns:
        Force magnitude (N)
    """
    force = 0.5 * coefficient * density * area * velocity**2
    return force


class HydrodynamicSurface:
    """
    Represents a hydrodynamic surface (sail, rudder, keel) with lift and drag characteristics.
    """

    def __init__(self, coeff_lift, coeff_drag, angle_of_attack, area, density_fluid):
        """
        Initialize a hydrodynamic surface.

        Args:
            coeff_lift: Array of lift coefficients vs angle of attack
            coeff_drag: Array of drag coefficients vs angle of attack
            angle_of_attack: Array of angles of attack (rad)
            area: Surface area (m²)
            density_fluid: Fluid density (kg/m³)
        """
        self.coefficient_lift = coeff_lift
        self.coefficient_drag = coeff_drag
        self.angle_attack = angle_of_attack
        self.area = area
        self.density_fluid = density_fluid

    def get_forces(self, attack_angle, velocity):
        """
        Calculate lift and drag forces for a given angle of attack and velocity.

        Args:
            attack_angle: Angle of attack (rad)
            velocity: Flow velocity magnitude (m/s)

        Returns:
            tuple: (lift_force, drag_force) in N
        """
        # Normalize angle to [-π, π]
        normalized_angle = np.angle(np.exp(1j * attack_angle))

        # Interpolate coefficients for this angle of attack
        lift_coeff = np.interp(normalized_angle, self.angle_attack, self.coefficient_lift)
        drag_coeff = np.interp(normalized_angle, self.angle_attack, self.coefficient_drag)

        # Calculate forces
        lift_force = calculate_lift_drag_force(lift_coeff, self.density_fluid, self.area, velocity)
        drag_force = calculate_lift_drag_force(drag_coeff, self.density_fluid, self.area, velocity)

        return lift_force, drag_force


def _eval_data(velocity, heading, turn_rate, rudder_angle):
    """
    Evaluate hydrodynamic forces and moments on the saildrone.

    Args:
        velocity: Velocity vector [vx, vy] (m/s)
        heading: Drone heading angle (rad)
        turn_rate: Angular velocity (rad/s)
        rudder_angle: Rudder deflection angle (rad)

    Returns:
        tuple: (net_force_vector, net_moment)
            - net_force_vector: [fx, fy] in N
            - net_moment: Moment about z-axis in N·m
    """
    # Constants
    FLUID_DENSITY = 1023.6  # kg/m³ (seawater)
    ROTATIONAL_DAMPING = 1000  # Rotational damping coefficient
    SAIL_AREA = 3.0  # m²
    RUDDER_AREA = 0.1  # m²
    KEEL_AREA = 0.25  # m²
    MOMENT_ARM_RUDDER = -3.0  # m (negative indicates aft of center)

    # Define angle of attack range for aerodynamic coefficients
    angles = np.deg2rad(np.arange(-180, 180.1, 0.1))

    # Lift coefficient model for sail/rudder
    lift_coefficient = 1.5 * np.sin(2 * angles + np.sin(2 * angles) / 2)

    # Drag coefficient model for sail/rudder
    drag_coefficient = 1 - np.cos(2 * angles)

    # Keel has no lift, only drag
    keel_lift_coefficient = np.zeros_like(drag_coefficient)

    # Keel drag coefficient (different model)
    keel_drag_coefficient = 1.5 - 0.5 * np.cos(2 * angles)
    # Lower drag when keel is aligned with flow
    low_angle_indices = np.where(np.abs(angles) < np.pi / 2)
    keel_drag_coefficient[low_angle_indices] = 1.25 - 0.75 * np.cos(2 * angles[low_angle_indices])

    # Create hydrodynamic surface objects
    sail = HydrodynamicSurface(lift_coefficient, drag_coefficient, angles, SAIL_AREA, FLUID_DENSITY)
    rudder = HydrodynamicSurface(lift_coefficient, drag_coefficient, angles, RUDDER_AREA, FLUID_DENSITY)
    keel = HydrodynamicSurface(keel_lift_coefficient, keel_drag_coefficient, angles, KEEL_AREA, FLUID_DENSITY)

    # Calculate drone orientation vectors
    heading_vector = np.array([np.cos(heading), np.sin(heading)])
    perpendicular_vector = np.array(np.cross(
        np.array([heading_vector[0], heading_vector[1], 0]),
        np.array([0, 0, -1])
    ))
    perpendicular_vector = perpendicular_vector[0:2]

    # Calculate flow velocity relative to drone
    velocity_magnitude = np.linalg.norm(velocity)
    flow_velocity = -velocity  # Flow is opposite to drone velocity

    # Calculate flow direction
    if velocity_magnitude == 0:
        flow_direction = np.array([0, 1])  # Default direction when stationary
    else:
        flow_direction = flow_velocity / np.linalg.norm(flow_velocity)

    # Perpendicular to flow direction
    flow_perpendicular = np.array(np.cross(
        np.array([-flow_direction[0], -flow_direction[1], 0]),
        np.array([0, 0, -1])
    ))
    flow_perpendicular = flow_perpendicular[0:2]

    # === SAIL FORCES ===
    # Angle of attack for sail (relative to heading)
    sail_angle_of_attack = np.arctan2(heading_vector[1], heading_vector[0]) - \
                          np.arctan2(-flow_direction[1], -flow_direction[0])

    sail_lift, sail_drag = sail.get_forces(sail_angle_of_attack, velocity_magnitude)

    # Lift acts perpendicular to flow, drag acts along flow
    sail_lift_force = flow_perpendicular * sail_lift
    sail_drag_force = flow_direction * sail_drag

    # === RUDDER FORCES ===
    # Rudder orientation (heading + rudder angle)
    rudder_vector = np.array([np.cos(heading + rudder_angle), np.sin(heading + rudder_angle)])

    # Angle of attack for rudder
    rudder_angle_of_attack = np.arctan2(rudder_vector[1], rudder_vector[0]) - \
                            np.arctan2(-flow_direction[1], -flow_direction[0])

    rudder_lift, rudder_drag = rudder.get_forces(rudder_angle_of_attack, velocity_magnitude)

    rudder_lift_force = flow_perpendicular * rudder_lift
    rudder_drag_force = flow_direction * rudder_drag

    # === KEEL FORCES ===
    keel_lift, keel_drag = keel.get_forces(sail_angle_of_attack, velocity_magnitude)

    keel_drag_force = flow_direction * keel_drag

    # === NET FORCES ===
    net_force = rudder_lift_force + rudder_drag_force + sail_lift_force + sail_drag_force + keel_drag_force

    # === NET MOMENTS ===
    # Moment from rudder lift
    rudder_lift_moment = np.dot(rudder_lift_force, perpendicular_vector)

    # Moment from rudder drag
    rudder_drag_moment = np.dot(rudder_drag_force, perpendicular_vector)

    # Total rudder moment with moment arm
    rudder_moment = MOMENT_ARM_RUDDER * (rudder_lift_moment + rudder_drag_moment)

    # Rotational damping moment
    damping_moment = -ROTATIONAL_DAMPING * turn_rate

    # Total moment
    net_moment = rudder_moment + damping_moment

    return net_force, net_moment

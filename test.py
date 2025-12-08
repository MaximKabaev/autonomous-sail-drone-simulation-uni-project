import numpy as np
import saildrone_hydro

# Moving straight up, heading up, no turn, no rudder
v = np.array([0, 2.9])
heading = np.pi/2
turn_rate = 0
rudder = 0

force, torque = saildrone_hydro.get_vals(v, heading, turn_rate, rudder)
print(f"Hydro torque (straight): {torque:.2f} N·m")

# Now with small clockwise turn rate
turn_rate_cw = -0.01  # clockwise
force2, torque2 = saildrone_hydro.get_vals(v, heading, turn_rate_cw, rudder)
print(f"Hydro torque (turning CW): {torque2:.2f} N·m")
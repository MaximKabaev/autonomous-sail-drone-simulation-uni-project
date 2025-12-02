import numpy as np
import matplotlib.pyplot as plt
import ode

def test_kp(kp, waypoints, wind):
    s0 = np.array([[0.0], [0.0], [np.pi/2], [0.0], [2.9], [0.0]])
    controller = ode.AutonomousController(waypoints, wind, Kp_rudder=kp, lookahead_distance=50)
    drone = ode.saildrone(controller=controller)
    t, s = drone.simulate_course(0, 0.1, s0, plot=False)
    
    # Time to complete
    completion_time = t[-1]
    
    # Average heading error
    heading_error = np.array(controller.control_history['heading_error'])
    avg_error = np.mean(np.abs(heading_error))
    
    # Total distance traveled
    dx = np.diff(s[0, :])
    dy = np.diff(s[1, :])
    total_distance = np.sum(np.sqrt(dx**2 + dy**2))
    
    # Rudder wear (total rudder angle changes)
    rudder_angles = np.array(controller.control_history['rudder_actual'])
    rudder_wear = np.sum(np.abs(np.diff(rudder_angles)))
    
    return completion_time, avg_error, total_distance, rudder_wear

# waypoints = [(0, 200), (-500, 500)]
waypoints = [(0, 200), (0, 400)]
wind = (-6.7, 0)
kp_values = np.linspace(0.05, 4.0, 100)

times = []
errors = []
distances = []
wears = []

print(f"{'Kp':<8} {'Time(s)':<10} {'Avg Err(°)':<12} {'Distance(m)':<14} {'Rudder Wear':<12}")
print("-" * 65)

for kp in kp_values:
    try:
        time, error, distance, wear = test_kp(kp, waypoints, wind)
        times.append(time)
        errors.append(np.rad2deg(error))
        distances.append(distance)
        wears.append(wear)
        print(f"{kp:<8.3f} {time:<10.1f} {np.rad2deg(error):<12.3f} {distance:<14.1f} {wear:<12.3f}")
    except Exception as e:
        print(f"{kp:<8.3f} ERROR: {str(e)[:30]}")

# Find optimal values
opt_time_idx = np.argmin(times)
opt_error_idx = np.argmin(errors)
opt_distance_idx = np.argmin(distances)
opt_wear_idx = np.argmin(wears)

# Plot results
fig, axes = plt.subplots(2, 2, figsize=(12, 10))

axes[0, 0].plot(kp_values[:len(times)], times, 'bo-', linewidth=2)
axes[0, 0].axvline(kp_values[opt_time_idx], color='r', linestyle='--', linewidth=2, label=f'Optimal: {kp_values[opt_time_idx]:.3f}')
axes[0, 0].plot(kp_values[opt_time_idx], times[opt_time_idx], 'r*', markersize=15)
axes[0, 0].set_xlabel('Kp')
axes[0, 0].set_ylabel('Completion Time (s)')
axes[0, 0].set_title('Time to Complete vs Kp')
axes[0, 0].legend()
axes[0, 0].grid(True, alpha=0.3)

axes[0, 1].plot(kp_values[:len(errors)], errors, 'ro-', linewidth=2)
axes[0, 1].axvline(kp_values[opt_error_idx], color='g', linestyle='--', linewidth=2, label=f'Optimal: {kp_values[opt_error_idx]:.3f}')
axes[0, 1].plot(kp_values[opt_error_idx], errors[opt_error_idx], 'g*', markersize=15)
axes[0, 1].set_xlabel('Kp')
axes[0, 1].set_ylabel('Average Error (degrees)')
axes[0, 1].set_title('Average Heading Error vs Kp')
axes[0, 1].legend()
axes[0, 1].grid(True, alpha=0.3)

axes[1, 0].plot(kp_values[:len(distances)], distances, 'go-', linewidth=2)
axes[1, 0].axvline(kp_values[opt_distance_idx], color='b', linestyle='--', linewidth=2, label=f'Optimal: {kp_values[opt_distance_idx]:.3f}')
axes[1, 0].plot(kp_values[opt_distance_idx], distances[opt_distance_idx], 'b*', markersize=15)
axes[1, 0].set_xlabel('Kp')
axes[1, 0].set_ylabel('Total Distance (m)')
axes[1, 0].set_title('Total Distance Traveled vs Kp')
axes[1, 0].legend()
axes[1, 0].grid(True, alpha=0.3)

axes[1, 1].plot(kp_values[:len(wears)], wears, 'mo-', linewidth=2)
axes[1, 1].axvline(kp_values[opt_wear_idx], color='c', linestyle='--', linewidth=2, label=f'Optimal: {kp_values[opt_wear_idx]:.3f}')
axes[1, 1].plot(kp_values[opt_wear_idx], wears[opt_wear_idx], 'c*', markersize=15)
axes[1, 1].set_xlabel('Kp')
axes[1, 1].set_ylabel('Rudder Wear (rad)')
axes[1, 1].set_title('Rudder Wear vs Kp')
axes[1, 1].legend()
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

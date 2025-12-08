import numpy as np
import matplotlib.pyplot as plt

def c_d(alpha):
    return 1 - np.cos(2 * alpha)

def c_l(alpha):
    # Adjust this to match your actual equation
    return 1.5 * np.sin(2 * alpha) + 0.5 * np.sin(2 * alpha)

def driving_force(alpha, awa, V_aw, rho=1.225, A=20.0):
    """
    alpha: angle of attack (radians)
    awa: apparent wind angle relative to bow (radians)
    V_aw: apparent wind speed (m/s)
    A: sail area (m^2)
    """
    cl = c_l(alpha)
    cd = c_d(alpha)
    
    q = 0.5 * rho * V_aw**2 * A  # dynamic pressure * area
    
    L = q * cl
    D = q * cd
    
    # Lift is perpendicular to apparent wind, drag is parallel
    # Project onto boat's forward direction
    # AWA is angle of apparent wind from bow
    # Wind comes FROM awa, so lift acts at (awa - 90°) from bow
    
    F_drive = L * np.sin(awa) - D * np.cos(awa)
    F_heel = L * np.cos(awa) + D * np.sin(awa)
    
    return F_drive, F_heel

def optimal_sail_angle(awa, V_aw, alpha_min=0.01, alpha_max=np.pi/2):
    """
    Find the angle of attack that maximizes driving force.
    Returns optimal alpha and corresponding sail angle (delta_s = awa - alpha)
    """
    alphas = np.linspace(alpha_min, alpha_max, 500)
    forces = [driving_force(a, awa, V_aw)[0] for a in alphas]
    
    best_idx = np.argmax(forces)
    best_alpha = alphas[best_idx]
    best_force = forces[best_idx]
    
    # Sail angle relative to centerline
    sail_angle = awa - best_alpha
    
    # Sail can't cross centerline (no negative angles) or go past 90°
    sail_angle = np.clip(sail_angle, 0, np.pi/2)
    
    return best_alpha, sail_angle, best_force

# Generate polar diagram
awa_range = np.linspace(np.deg2rad(20), np.deg2rad(180), 100)  # 20° to 180°
wind_speeds = [5, 10, 15, 20]  # m/s

fig, axes = plt.subplots(1, 3, figsize=(15, 5))

# Plot 1: Optimal driving force vs AWA (polar style)
ax1 = plt.subplot(131, projection='polar')
for V_aw in wind_speeds:
    forces = []
    for awa in awa_range:
        _, _, f = optimal_sail_angle(awa, V_aw)
        forces.append(max(f, 0))  # No negative driving force
    ax1.plot(awa_range, forces, label=f'{V_aw} m/s')
ax1.set_theta_zero_location('N')
ax1.set_theta_direction(-1)
ax1.set_thetamin(0)
ax1.set_thetamax(180)
ax1.set_title('Driving Force Polar')
ax1.legend(loc='lower left')

# Plot 2: Optimal sail angle vs AWA
ax2 = axes[1]
for V_aw in wind_speeds:
    sail_angles = []
    for awa in awa_range:
        _, sa, _ = optimal_sail_angle(awa, V_aw)
        sail_angles.append(np.rad2deg(sa))
    ax2.plot(np.rad2deg(awa_range), sail_angles, label=f'{V_aw} m/s')
ax2.set_xlabel('Apparent Wind Angle (deg)')
ax2.set_ylabel('Optimal Sail Angle from Centerline (deg)')
ax2.set_title('Optimal Sail Trim')
ax2.legend()
ax2.grid(True)

# Plot 3: Optimal angle of attack vs AWA
ax3 = axes[2]
for V_aw in wind_speeds:
    alphas = []
    for awa in awa_range:
        a, _, _ = optimal_sail_angle(awa, V_aw)
        alphas.append(np.rad2deg(a))
    ax3.plot(np.rad2deg(awa_range), alphas, label=f'{V_aw} m/s')
ax3.set_xlabel('Apparent Wind Angle (deg)')
ax3.set_ylabel('Optimal Angle of Attack (deg)')
ax3.set_title('Optimal α')
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.savefig('sail_polar.png', dpi=150)
plt.show()

# Print a lookup table
print("\nOptimal Sail Angle Lookup Table (V_aw = 10 m/s):")
print("AWA (deg) | Sail Angle (deg) | Alpha (deg) | F_drive (N)")
print("-" * 55)
for awa_deg in range(30, 181, 15):
    awa = np.deg2rad(awa_deg)
    alpha, sail_angle, force = optimal_sail_angle(awa, 10.0)
    print(f"{awa_deg:^9} | {np.rad2deg(sail_angle):^16.1f} | {np.rad2deg(alpha):^11.1f} | {force:^11.1f}")
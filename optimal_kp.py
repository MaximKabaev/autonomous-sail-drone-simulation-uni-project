import ode
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':

    s0 = np.array([
                [0.0],
                [0.0],
                [np.pi/2],
                [0.0],
                [2.9],
                [0.0]])
    
    waypoints = [
        (200, 400)
    ]

    kp_to_number_of_oscillations = []

    for i in range(130, 140):  # Test Kp_rudder from 1.30 to 1.39

        controller = ode.AutonomousController(
            waypoints=waypoints,
            wind=(-6.7, 0),  # Use actual wind vector
            Kp_rudder=i*0.001,  # Higher gain for stability (tune this value)
            lookahead_distance=1
        )
        
        saildrone = ode.saildrone(controller=controller)
        
        t, s = saildrone.simulate_course(0, 0.1, s0, False)

        history = controller.control_history
        actual_tmax = t[-1]
        time_steps = np.linspace(0, actual_tmax, len(history['heading_error']))

        heading_error_deg = np.rad2deg(history['heading_error'])
        zero_crossings = np.where(np.diff(np.sign(heading_error_deg)))[0]
        kp_to_number_of_oscillations.append([i*0.001, len(zero_crossings)])
        print(f"Number of zero crossings: {len(zero_crossings)} with {i*0.001} Kp_rudder")

        # plt.figure(figsize=(10, 5))
        # plt.plot(time_steps, np.rad2deg(history['heading_error']), 'b-', linewidth=2)


    plt.figure(figsize=(8, 4))
    x = [i[0] for i in kp_to_number_of_oscillations]
    y = [i[1] for i in kp_to_number_of_oscillations]
    points, = plt.plot(x, y, marker='o', linestyle='None')  # Only dots

    plt.axhline(y=2, color='red', linestyle='--', linewidth=2)
    plt.xlabel('Kp_rudder')
    plt.ylabel('Number of Zero Crossings (Oscillations)')
    plt.title('Oscillations vs Kp_rudder')
    plt.grid(True)

    def on_pick(event):
        ind = event.ind[0]
        kp = x[ind]
        osc = y[ind]
        plt.gca().annotate(f'({kp:.4f}, {osc})', (kp, osc),
                        textcoords="offset points", xytext=(10,10),
                        ha='center', fontsize=9, color='blue',
                        arrowprops=dict(arrowstyle="->", color='blue'))
        plt.draw()

    points.set_picker(5)  # 5 points tolerance

    plt.gcf().canvas.mpl_connect('pick_event', on_pick)
    plt.show()

    
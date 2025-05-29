import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.optimize import curve_fit
from scipy.signal import savgol_filter

def fix_cart_pose(cart_poses):
    return (cart_poses - cart_poses[0])

def fix_pole_velocity(velocity):
    for i in range(len(velocity)):
        if abs(velocity[i]) > 250:
            velocity[i] = velocity[i-1]
    return velocity

def fix_pole_angle(angles, fix_steady_pos=True):
    pole_rad = np.unwrap(angles)

    if fix_steady_pos:
        pole_rad -= np.mean(pole_rad[-50])
    return pole_rad #pole_rad_centered

def load_and_plot(filename):
    data = np.load(filename)

    # Unpack columns
    time_us     = data[:, 0]  # microseconds
    dt_us       = data[:, 1]
    cart_m      = fix_cart_pose(data[:, 2])
    cart_speed  = data[:, 3]
    pole_rad    = fix_pole_angle(data[:, 4])
    pole_speed  = fix_pole_velocity(data[:, 5])
    control     = data[:, 6]

    # # Get unique control values
    # unique_controls = np.unique(control)

    # # Create a figure with subplots
    # fig, axes = plt.subplots(len(unique_controls), 1, figsize=(12, 6*len(unique_controls)))

    # # If only one control value, make axes a list for consistency
    # if len(unique_controls) == 1:
    #     axes = [axes]

    # for ax, control_val in zip(axes, unique_controls):
    #     mask = (control == control_val)
    #     speeds = cart_speed[mask]
    #     time_points = np.arange(len(speeds)) * dt_us[mask][0]  # or use time_us[mask]

    #     mean_speeds = np.mean(speeds[30:-30]) 

    #     ax.plot(time_points, speeds)
    #     ax.plot([time_points[0], time_points[-1]], [mean_speeds, mean_speeds], 'r--')
    #     ax.set_title(f'Control = {control_val:.2f}')
    #     ax.set_xlabel('Time (μs)')
    #     ax.set_ylabel('Cart Speed')
    #     ax.grid(True)
    #     print(f"Control_val {control_val}: {mean_speeds}")


    # plt.tight_layout()
    # plt.show()
    # asdasd

    energy = []
    for i in range(len(cart_speed)):
        L = 0.227
        g = 9.81
        m = 0.3
        pot_energy =  0.5 * m * L**2 * pole_speed[i]**2
        kin_energy =  m * g * L * (1 - np.cos(pole_rad[i]))
        energy.append(pot_energy + kin_energy)

    dt = np.mean(dt_us) / 1e6
    cart_accel = savgol_filter(cart_speed, 11, 2, deriv=1, delta=dt)
    pole_accel = savgol_filter(pole_speed, 11, 2, deriv=1, delta=dt)

    # Convert time to seconds
    time = (time_us - time_us[0]) / 1e6

    # Plot
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # --- Cart Position & Speed with Twin Y Axes ---
    ax0 = axs[0]
    ax1 = ax0.twinx()

    lns1 = ax0.plot(time, cart_m, 'b-', label='Cart Position (m)')
    lns2 = ax1.plot(time, cart_speed, 'g--', label='Cart Speed (m/s)')
    lns3 = ax1.plot(time, cart_accel, 'r--', label='Cart Accel (m/s^2)')

    ax0.set_ylabel("Position (m)", color='b')
    ax1.set_ylabel("Speed (m/s) / Accel (m/s^2)", color='g')
    ax0.tick_params(axis='y', labelcolor='b')
    ax1.tick_params(axis='y', labelcolor='g')

    lns = lns1 + lns2 + lns3
    labels = [l.get_label() for l in lns]
    ax0.legend(lns, labels, loc='upper right')

    axs[1].plot(time, pole_rad, label='Pole Angle (rad)')
    axs[1].plot(time, pole_speed, label='Pole Speed (rad/s)', linestyle='--')
    axs[1].plot(time, energy, label='Energy', linestyle='--')
    axs[1].legend()
    axs[1].set_ylabel("Pole")

    plt.suptitle("Cart-Pole Recording")
    plt.tight_layout()
    png_filename = filename.replace('.npy', '.png')
    plt.savefig(png_filename, dpi=300)
    print(f"Plot saved as: {png_filename}")

def load_data_for_peaks(filename):
    data = np.load(filename)

    # Unpack columns
    time_us     = data[:, 0]  # microseconds
    dt_us       = data[:, 1]
    cart_m      = fix_cart_pose(data[:, 2])
    cart_speed  = data[:, 3]
    pole_rad    = fix_pole_angle(data[:, 4])
    pole_speed  = fix_pole_velocity(data[:, 5])
    control     = data[:, 6]

    # Convert time to seconds
    time = (time_us - time_us[0]) / 1e6

    # Find peaks of the pole angle (absolute peaks for damping envelope)
    peaks_idx, _ = find_peaks(np.abs(pole_rad), distance=10)
    peaks_t = time[peaks_idx]
    peaks_val = np.abs(pole_rad[peaks_idx])
        
    # Define a decay envelope model with both viscous and Coulomb-like damping
    # A * exp(-b * t) + c * (1 - exp(-d * t)) → simplified as a damped exponential with offset
    def viscous_coulomb_decay(t, A, b, c, d):
        return A * np.exp(-b * t) + c * (1 - np.exp(-d * t))

    # Fit using the detected peaks (magnitude only for now, signed could be added later)
    popt_vc, _ = curve_fit(viscous_coulomb_decay, peaks_t, peaks_val, p0=[1.5, 0.1, 0.2, 0.5], maxfev=20000)


    plt.figure(figsize=(10, 6))
    plt.plot(time, np.abs(pole_rad), label="|Pole Angle|", alpha=0.5)
    plt.plot(peaks_t, peaks_val, 'ro', label="Peaks")
    plt.plot(peaks_t, viscous_coulomb_decay(peaks_t, *popt_vc), 'k--', label="Viscous + Coulomb Fit")

    plt.title("Pole Angle Peaks and Exponential Decay Envelope")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle Magnitude (rad)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Estimate frequency from time between zero-crossings
    zero_crossings = np.where(np.diff(np.sign(pole_rad)))[0]
    periods = np.diff(time[zero_crossings[::2]])  # every second crossing is one full period
    avg_period = np.mean(periods)
    frequency_hz = 1 / avg_period

    # Pack results
    result = {
        "Estimated Frequency (Hz)": f"{frequency_hz} +- {np.std(periods)}",
        "Decay Envelope Parameters": popt_vc
    }
    print(f"{filename.split('/')[-1]}: {result}")

if __name__ == "__main__":
    # load_and_plot("Project 4 Cart-Pole/recorded_data/record_1748441081.npy")
    # load_and_plot("Project 4 Cart-Pole/recorded_data/record_1748441130.npy")
    # load_and_plot("Project 4 Cart-Pole/recorded_data/record_1748441174.npy")
    # load_and_plot("Project 4 Cart-Pole/recorded_data/record_1748441220.npy")

    # load_and_plot("Project 4 Cart-Pole/controlled_data/record_1748458654.npy")
    # load_and_plot("Project 4 Cart-Pole/controlled_data/record_1748460593.npy")
    load_and_plot("Project 4 Cart-Pole/new_recorded_data/record_1748487383.npy")

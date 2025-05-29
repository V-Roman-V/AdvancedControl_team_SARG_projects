import matplotlib.pyplot as plt
import numpy as np

# Calibration data (control: speed)
calibration = {
    -600.0: 0.5583022702702702,
    -550.0: 0.4926020694444444,
    -500.0: 0.3043371090909091,
    -450.0: 0.27637998823529414,
    -400.0: 0.2470016288209607,
    -350.0: 0.1681537471590909,
    -300.0: 0.13894805614973263,
    -250.0: 0.1013793660130719,
    -200.0: 0.042281848111025436,
    0.0: 0.0,
    200.0: -0.04694190606319385,
    250.0: -0.12433030107526882,
    300.0: -0.19817649305555554,
    350.0: -0.24244048,
    400.0: -0.32735501226993863,
    450.0: -0.4098146851851852,
    500.0: -0.47590564341085273,
    550.0: -0.5588143469387755,
    600.0: -0.6530022999999999,
}

# Linear model parameters
calib_configs_1 = [-0.0011, -0.10] # Params for control < 0
calib_configs_2 = [-0.0012, 0.10]  # Params for control > 0

def control_to_speed(control):
    k,b = calib_configs_1 if control < 0 else calib_configs_2
    speed = k * control + b
    return max(speed, 0) if control < 0 else min(speed, 0)

def speed_to_control(speed):
    if abs(speed) < 0.001:  # Deadzone to avoid noise near zero
        return 0
    k,b = calib_configs_1 if speed > 0 else calib_configs_2
    return (speed - b) / k  # Inverse for positive speed

# Generate smooth curves
control_values = np.linspace(-600, 600, 1000)  # Fine grid for smooth plot
speed_values = np.linspace(-0.7, 0.7, 1000)   # Extend slightly beyond data range

# Create figure with two subplots (side by side)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

# --- Plot 1: Control to Speed ---
ax1.scatter(
    calibration.keys(), calibration.values(), 
    color='red', label='Calibration Data', zorder=3
)
ax1.plot(
    control_values, 
    [control_to_speed(v) for v in control_values], 
    'b-', linewidth=2, alpha=0.7, 
    label='Linear Model'
)
ax1.set_title("Control Input → Speed Output", fontsize=12)
ax1.set_xlabel("Control Value", fontsize=10)
ax1.set_ylabel("Speed Output", fontsize=10)
ax1.grid(True, linestyle='--', alpha=0.5)
ax1.axhline(0, color='black', linestyle='-', linewidth=0.5)
ax1.axvline(0, color='black', linestyle='-', linewidth=0.5)
ax1.legend()

# --- Plot 2: Speed to Control (Inverse) ---
ax2.scatter(
    calibration.values(), calibration.keys(), 
    color='red', label='Calibration Data', zorder=3
)
ax2.plot(
    speed_values,
    [speed_to_control(v) for v in speed_values],
    'g-', linewidth=2, alpha=0.7, 
    label='Inverse Model'
)
ax2.set_title("Speed Output → Control Input (Inverse)", fontsize=12)
ax2.set_xlabel("Speed Output", fontsize=10)
ax2.set_ylabel("Control Value", fontsize=10)
ax2.grid(True, linestyle='--', alpha=0.5)
ax2.axhline(0, color='black', linestyle='-', linewidth=0.5)
ax2.axvline(0, color='black', linestyle='-', linewidth=0.5)
ax2.legend()

plt.tight_layout()
plt.show()
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.signal import savgol_filter
from tqdm import tqdm

# Constants
g = 9.81  # gravitational acceleration

# Initial parameter guess: M, m, l, b_c, f_c, b_p, f_p
initial_params = np.array([
    4.5,       # M: cart mass (kg)
    0.210,     # m: pole mass (kg)
    0.196,     # l: pole length (m)
    0.8,       # b_c: viscous friction (cart)
    0.9,       # f_c: Coulomb friction (cart)
    0.0016,    # b_p: viscous friction (pole)
    0.0062,     # f_p: Coulomb friction (pole)
    100         # K_p: Virtual Force  
])


# Linear model parameters
calib_configs_1 = [-0.0014035, -0.2821036] # Params for control < 0
calib_configs_2 = [-0.00164, 0.334]  # Params for control > 0

def control_to_speed(control):
    k,b = calib_configs_1 if control < 0 else calib_configs_2
    speed = k * control + b
    return max(speed, 0) if control < 0 else min(speed, 0)

def speed_to_control(speed):
    if abs(speed) < 0.05:  # Deadzone to avoid noise near zero
        return 0
    k,b = calib_configs_1 if speed > 0 else calib_configs_2
    return (speed - b) / k  # Inverse for positive speed

def cart_pole_dynamics(state, params, u_cmd=0.0):
    x, x_dot, theta, theta_dot = state
    M, m, L, b_c, f_c, b_p, f_p, K_pf = params

    # Intermediate terms
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    sgn_x_dot = np.sign(x_dot)
    sgn_theta_dot = np.sign(theta_dot)

    # Friction forces
    F_fric = b_c * x_dot + f_c * sgn_x_dot
    T_fric = b_p * theta_dot + f_p * sgn_theta_dot

    # Virtual Force
    u_f = K_pf * (control_to_speed(u_cmd) - x_dot)

    D = M + m
    mlcos = m * L * cos_theta

    # Compute effective inertia term for theta equation
    alpha = m * L**2 - (mlcos**2) / D

    # Numerator of angular acceleration
    beta = (
        -m * g * L * sin_theta
        - (mlcos / D) * (-M*u_f + F_fric + m * L * theta_dot**2 * sin_theta)
        - T_fric
    )

    # Angular acceleration
    theta_ddot = beta / alpha

    # Linear acceleration
    pole_force = mlcos * theta_ddot - m * L * theta_dot**2 * sin_theta
    x_ddot = u_f + pole_force / 100

    return np.array([x_dot, x_ddot, theta_dot, theta_ddot])



def simulate_cart_pole(init_state, params, dt, steps, control):
    time = np.linspace(0, dt*steps, num=steps, endpoint=False)
    print(f"{time[:5] = } {time[-5: ]= }")
    state_array = [init_state]
    for u_cmd in tqdm(control[1:]):
        cur_state = state_array[-1]
        deriv = cart_pole_dynamics(cur_state, params, u_cmd)
        new_state = cur_state + deriv*dt
        state_array.append(new_state)
    return time, np.array(state_array)

# Loss function
def loss_function(params, states, derivs):
    total_loss = 0.0
    for s, d_true in zip(states, derivs):
        d_pred = cart_pole_dynamics(s, params)
        total_loss += np.sum((d_true - d_pred) ** 2)
    return total_loss / len(states)


def load_data(filename):
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
            pole_rad -= np.mean(pole_rad[-30])
        return pole_rad #pole_rad_centered

    data = np.load(filename)
    time_us     = data[:, 0]  # microseconds
    dt_us       = data[:, 1]
    cart_m      = fix_cart_pose(data[:, 2])
    cart_speed  = data[:, 3]
    pole_rad    = fix_pole_angle(data[:, 4])
    pole_speed  = fix_pole_velocity(data[:, 5])
    control     = data[:, 6]

    dt = np.mean(dt_us) / 1e6
    cart_accel = savgol_filter(cart_speed, 11, 2, deriv=1, delta=dt)
    pole_accel = savgol_filter(pole_speed, 11, 2, deriv=1, delta=dt)

    states = np.array([cart_m, cart_speed, pole_rad, pole_speed]).T
    derives = np.array([cart_speed, pole_speed, cart_accel, pole_accel]).T
    return dt, states, derives, control

def plot_state_comparising(time, state_data, state_sim):
    d_cart_m, d_cart_speed, d_pole_rad, d_pole_speed = state_data.T
    s_cart_m, s_cart_speed, s_pole_rad, s_pole_speed = state_sim.T
    
    # Plot
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # --- Cart Position & Speed with Twin Y Axes ---
    ax0 = axs[0]
    ax1 = ax0.twinx()

    lns1 = ax0.plot(time, d_cart_m, 'b--', label='Cart Position (m)')
    lns2 = ax1.plot(time, d_cart_speed, 'g--', label='Cart Speed (m/s)')
    lns3 = ax0.plot(time, s_cart_m, 'b-', label='Simulated Cart Position (m)', alpha=0.5)
    lns4 = ax1.plot(time, s_cart_speed, 'g-', label='Simulated Cart Speed (m/s)', alpha=0.5)


    ax0.set_ylabel("Position (m)", color='b')
    ax1.set_ylabel("Speed (m/s)", color='g')
    ax0.tick_params(axis='y', labelcolor='b')
    ax1.tick_params(axis='y', labelcolor='g')

    lns = lns1 + lns2 + lns3 + lns4
    labels = [l.get_label() for l in lns]
    ax0.legend(lns, labels, loc='upper right')

    axs[1].plot(time, d_pole_rad, color='b', label='Pole Angle (rad)', linestyle='--')
    axs[1].plot(time, d_pole_speed, color='orange', label='Pole Speed (rad/s)', linestyle='--')
    axs[1].plot(time, s_pole_rad, color='b', label='Simulated Pole Angle (rad)')
    axs[1].plot(time, s_pole_speed,  color='orange', label='Simulated Pole Speed (rad/s)')
    axs[1].legend()
    axs[1].set_ylabel("Pole")

    plt.suptitle("Cart-Pole Recording")
    plt.tight_layout()
    plt.show()

# Load data
# dt, state_data, deriv_data, control = load_data("Project 4 Cart-Pole/recorded_data/record_1748441130.npy")
dt, state_data, deriv_data, control = load_data("Project 4 Cart-Pole/controlled_data/record_1748458654.npy")

# Simulate initial system
times, state_sim = simulate_cart_pole(state_data[0], initial_params, dt, len(state_data), control)
plot_state_comparising(times, state_data, state_sim)

# res = minimize(loss_function, initial_params, args=(state_data, deriv_data), method='L-BFGS-B')
# print(res.x, res.fun)

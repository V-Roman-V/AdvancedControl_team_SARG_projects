import time
import numpy as np

from UART import UART
from cart_pole import State, CartPole
from converter import control_to_speed, speed_to_control
from MPC_controller import NMPCControllerSC, SwingNMPCControllerSC


def main():
    cartpole = CartPole()
    uart = UART()
    
    mpc_swing = SwingNMPCControllerSC(cartpole, horizon=5, dt=0.007, num_iterations=10)
    # mpc_st = NMPCControllerSC(cartpole, horizon=4, dt=0.007, num_iterations=10)
    # mpc_st = NMPCControllerSC(cartpole, horizon=4, dt=0.005, num_iterations=10)
    mpc_st = NMPCControllerSC(cartpole, horizon=4, dt=0.007, num_iterations=20)

    for i in range(50):
        state = uart.wait_until_state()
        print("     ",state)

    # Parameters
    Thetha_shift = 0.00     # shift thetha value
    Energy_swing_shift = 0.05 # shifts maximum energy condition
    Waiting_angle_rad = 0.16  # if inside this area we stabilize
    # ----
    current_action = 0 # 0 - swing, 1 - wait, 2 - stable
    print("Start work!")
    uart.send_cart_velocity(0.8, verbose=True)
    time.sleep(0.10)
    uart.send_cart_velocity(-0.8, verbose=True)
    time.sleep(0.10)
    try:
        while True:
            # read state
            state = uart.wait_until_state()

            # Correcting angle value
            state.theta += Thetha_shift
            print("     ",state)

            # # Safety
            if abs(state.x) > 0.25:
                uart.send_cart_velocity(0)
                print(f"Reach border", state)
                time.sleep(1)
                break 

            M, m, L, b_c, f_c, b_p, f_p, K_pf = cartpole.params
            pot_energy =  0.5 * m * L**2 * state.theta_dot**2
            kin_energy =  m * cartpole.g * L * (1 - np.cos(state.theta))
            current_energy = pot_energy + kin_energy

            high_energy_cond = current_energy > (m * cartpole.g * L * 2 - Energy_swing_shift)
            theta_up_cond = abs(state.theta - np.pi) < Waiting_angle_rad

            if not high_energy_cond and not theta_up_cond:
                current_action = 0 # Swing-up
            elif high_energy_cond and not theta_up_cond:
                current_action = 1 # Waiting
            elif theta_up_cond:
                current_action = 2 # Stabilazing

            # MPC control
            start = time.time()
            u_opt = 0
            if current_action == 0:
                u_opt = mpc_swing.solve_mpc(state)
            elif current_action == 2:
                u_opt = mpc_st.solve_mpc(state)
                u_opt += 0.4 * state.x

            print(f"Optimal Control [{current_action}]:", u_opt, f" dt = {(time.time() - start):.3f}")

            # send velocity
            uart.send_cart_velocity(u_opt, verbose=True)
    except KeyboardInterrupt:
        print("\nCtrl-C detected. Sending zero speed and exiting...")
        uart.send_cart_velocity(0)
        time.sleep(1)

if __name__ == "__main__":
    main()

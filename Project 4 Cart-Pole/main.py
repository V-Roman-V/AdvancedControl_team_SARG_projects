import time
import numpy as np

from UART import UART
from cart_pole import State, CartPole
from converter import control_to_speed, speed_to_control
from MPC_controller import NMPCControllerSC


def main():
    cartpole = CartPole()
    uart = UART()
    
    mpc = NMPCControllerSC(cartpole, horizon=5, dt=0.007, num_iterations=10)

    for i in range(50):
        state = uart.wait_until_state()
        print("     ",state)

    print("Start work!")
    try:
        while True:
            # read state
            state = uart.wait_until_state()
            print("     ",state)

            # Safety
            if abs(state.x) > 0.25:
                uart.send_cart_velocity(0)
                print(f"Reach border", state)
                time.sleep(1)
                break 

            # MPC control
            start = time.time()
            u_opt = mpc.solve_mpc(state)
            
            prev_error = 0
            if abs(state.x) > 0.05:
                kp = 0.4
                kd = -0.2
                error = np.sign(state.x) * (np.abs(state.x) - 0.05)
                error_diff = prev_error - error
                prev_error = error
                u_opt += kp * error + kd * error_diff
            else:
                prev_error = 0
            print("Optimal Control:", u_opt, f" dt = {(time.time() - start):.3f}")

            # send velocity
            uart.send_cart_velocity(u_opt, verbose=True)
    except KeyboardInterrupt:
        print("\nCtrl-C detected. Sending zero speed and exiting...")
        uart.send_cart_velocity(0)
        time.sleep(1)

if __name__ == "__main__":
    main()

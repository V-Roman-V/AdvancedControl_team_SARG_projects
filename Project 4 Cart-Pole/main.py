import time
import numpy as np

from UART import UART
from cart_pole import State, CartPole
from converter import control_to_speed, speed_to_control
from MPC_controller import NMPCController

def main():
    cartpole = CartPole()
    uart = UART()
    mpc = NMPCController(cartpole, horizon=10, dt=0.007)

    while True:
        # read state
        state = uart.wait_until_state()
        print("     ",state)

        # Safety
        if abs(state.x) > 0.25:
            uart.send_cart_velocity(0)
            print(f"Reach border", state)
            break 

        # MPC control
        try:
            u_opt = mpc.compute_control(state)
            print("Optimal Control:", u_opt)
        except Exception as e:
            uart.send_cart_velocity(0)
            print(f"Error inside MPC: {e}")
            break 

        # send velocity
        uart.send_cart_velocity( u_opt, verbose=True)

if __name__ == "__main__":
    main()

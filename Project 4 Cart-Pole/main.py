import serial
import time
import struct
import numpy as np
import sys
import select

from UART import UART

def main():
    uart = UART()
    while True:
        state = uart.wait_until_state()
        print(f"[{state['timestamp']}] Δt={state['dt']:.3f}s | Cart: {state['cart_m']:.3f} m @ {state['cart_speed']:.3f} m/s | "f"Pole: {state['pole_rad']:.3f} rad @ {state['pole_speed']:.3f} rad/s | u={state['last_control']}")
        uart.send_cart_velocity(state['cart_speed'], verbose=True)

if __name__ == "__main__":
    main()

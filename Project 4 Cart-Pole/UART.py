import serial
import time
import struct
import numpy as np
import sys
import select
from converter import control_to_speed, speed_to_control

from cart_pole import State

# ----- Parser for incoming state -----
class UART:
    def __init__(self, port = '/dev/ttyUSB0', baudrate = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(0.1)
        print("UART connected\n")
        self.last_state: State = None

    def wait_until_state(self) -> State:
        while True:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            state = self._parse_state_line(line)
            if not state:
                # print("Got not state message:", repr(line))
                continue
            state = self._fix_state(state)
            self.last_state = state
            return state

    def send_cart_velocity(self, velocity, verbose=False):
        control = speed_to_control(velocity)
        self._send_cart_control(control, verbose)

    def _parse_state_line(self, line) -> State | None:
        try:
            if not line.startswith("S "):
                return None
            parts = line.strip().split()
            if len(parts) < 7:
                return None
            return State(
                timestamp = 1e-6 * int(parts[1]),
                dt = 1e-6 * int(parts[2]),
                x = float(parts[3]),
                x_dot = float(parts[4]),
                theta = float(parts[5]),
                theta_dot = float(parts[6]),
                old_ctrl = float(parts[7]),
            )
        except Exception as e:
            print(f"Error parsing: {e}")
            return None

    def _fix_state(self, state: State) -> State:
        if self.last_state is None:
            self.last_state = state
        pole_vel = state.theta_dot
        if pole_vel > 250:
            old_angle = self.last_state.theta
            new_angle = state.theta
            dt = state.dt
            angle_diff = (new_angle - old_angle + np.pi) % (2 * np.pi) - np.pi
            new_pole_vel = angle_diff / dt
            state.theta_dot = new_pole_vel
        return state

    def _send_cart_control(self, control, verbose=False):
        MAX_CTRL = 127
        scale_control = 0.1 # parameter inside STM
        control = int(max(-MAX_CTRL, min(MAX_CTRL, control*scale_control)))  # clamp to one byte
        self.ser.write(struct.pack('b', control))
        if verbose:
            print(f"Sent: {control}")

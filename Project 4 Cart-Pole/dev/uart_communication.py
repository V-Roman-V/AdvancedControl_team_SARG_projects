import serial
import time
import struct
import numpy as np
import sys
import select

# ----- Controller Parameters -----
KP = 200  # proportional gain (tune it experimentally)
MAX_VEL = 125  # maximum motor command to avoid overload

# ----- Parser for incoming state -----
def parse_state_line(line):
    try:
        if not line.startswith("S "):
            return None
        parts = line.strip().split()
        if len(parts) < 7:
            return None
        return {
            "timestamp": int(parts[1]),
            "dt": int(parts[2]),
            "cart_m": float(parts[3]),
            "cart_speed": float(parts[4]),
            "pole_rad": float(parts[5]),
            "pole_speed": float(parts[6]),
            "last_control": parts[7],
        }
    except Exception as e:
        print(f"Error parsing: {e}")
        return None

# ----- UART sender -----
def send_cart_velocity(ser, velocity):
    scale_velocity = 0.2 # parameter inside STM
    velocity = int(max(-MAX_VEL, min(MAX_VEL, velocity*scale_velocity)))  # clamp to safe bounds
    ser.write(struct.pack('b', velocity))
    print(f"Sent: {velocity}")


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

# ----- Main UART control loop -----
def main():
    port = '/dev/ttyUSB0'  # adjust if needed
    baudrate = 115200

    with serial.Serial(port, baudrate, timeout=1) as ser:
        ser.reset_input_buffer()
        time.sleep(0.1)
        print("Running controller over UART...\n")

        control = 0
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            # print("RAW:", repr(line))  # optional for debugging

            state = parse_state_line(line)
            if state:
                print(f"[{state['timestamp']}] Δt={state['dt']}us | Cart: {state['cart_m']:.3f} m @ {state['cart_speed']:.3f} m/s | "
                      f"Pole: {state['pole_rad']:.3f} rad @ {state['pole_speed']:.3f} rad/s | u={state['last_control']}")

                # control = speed_to_control(state['cart_speed'])
                # send_cart_velocity(ser, control)

            # Check for user input (poll every loop)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip()
                if cmd == 'q':
                    break
                else:
                    try:
                        speed = int(cmd)
                        if abs(speed) < MAX_VEL:
                            control = speed
                    except Exception:
                        pass 

if __name__ == "__main__":
    main()

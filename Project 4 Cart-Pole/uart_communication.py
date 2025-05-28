import serial
import time
import struct
import numpy as np

# ----- Controller Parameters -----
KP = 200  # proportional gain (tune it experimentally)
MAX_VEL = 30  # maximum motor command to avoid overload

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
    velocity = int(max(-MAX_VEL, min(MAX_VEL, velocity)))  # clamp to safe bounds
    ser.write(struct.pack('b', velocity))
    print(f"Sent: {velocity}")

# ----- Simple Proportional Controller -----
def compute_control(state):
    pole_angle = state['pole_rad']
    error = np.pi - pole_angle
    # u = -Kp * x
    return np.sign(error) * MAX_VEL

# ----- Main UART control loop -----
def main():
    port = '/dev/ttyUSB0'  # adjust if needed
    baudrate = 115200

    with serial.Serial(port, baudrate, timeout=1) as ser:
        ser.reset_input_buffer()
        time.sleep(0.1)
        print("Running P-controller over UART...\n")

        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            # print("RAW:", repr(line))  # optional for debugging

            state = parse_state_line(line)
            if state:
                print(f"[{state['timestamp']}] Δt={state['dt']}us | Cart: {state['cart_m']:.3f} m @ {state['cart_speed']:.3f} m/s | "
                      f"Pole: {state['pole_rad']:.3f} rad @ {state['pole_speed']:.3f} rad/s | u={state['last_control']}")

                # u = compute_control(state)
                # send_cart_velocity(ser, u)

if __name__ == "__main__":
    main()

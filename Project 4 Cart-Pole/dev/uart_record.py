import serial
import time
import numpy as np
import sys
import select
import struct

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
    velocity = -int(max(-MAX_VEL, min(MAX_VEL, velocity)))  # clamp to safe bounds
    ser.write(struct.pack('b', velocity))
    print(f"Sent: {velocity}")

# ----- Main UART control loop -----
def main():
    port = '/dev/ttyUSB0'
    baudrate = 115200

    recorded_data = []
    recording = False

    with serial.Serial(port, baudrate, timeout=1) as ser:
        ser.reset_input_buffer()
        time.sleep(0.1)
        print("Press ENTER to start/stop recording. Type 'q' to quit.\n")

        while True:
            # Non-blocking serial read
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                state = parse_state_line(line)
                if state:
                    # print(f"[{state['timestamp']}] Δt={state['dt']}us | Cart: {state['cart_m']:.3f} m @ {state['cart_speed']:.3f} m/s | "
                    #       f"Pole: {state['pole_rad']:.3f} rad @ {state['pole_speed']:.3f} rad/s | u={state['last_control']}")

                    if recording:
                        recorded_data.append([
                            state['timestamp'],
                            state['dt'],
                            state['cart_m'],
                            state['cart_speed'],
                            state['pole_rad'],
                            state['pole_speed'],
                            int(state['last_control'])
                        ])

            # Check for user input (poll every loop)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip()
                if cmd == 'q':
                    break
                elif cmd == '':
                    recording = not recording
                    print(f"{'Started' if recording else 'Stopped'} recording.")
                else:
                    try:
                        speed = int(cmd)
                        if abs(speed) < MAX_VEL:
                            send_cart_velocity(ser, speed)
                    except Exception:
                        pass 

    if recorded_data:
        data_np = np.array(recorded_data)
        filename = f"record_{int(time.time())}.npy"
        np.save(filename, data_np)
        print(f"Saved {len(recorded_data)} samples to {filename}.")


if __name__ == "__main__":
    main()

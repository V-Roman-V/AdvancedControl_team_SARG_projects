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
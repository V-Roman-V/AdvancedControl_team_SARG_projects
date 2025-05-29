# Linear model parameters
calib_configs_1 = [-0.0011, -0.10] # Params for control < 0
calib_configs_2 = [-0.0012, 0.10]  # Params for control > 0

def control_to_speed(control):
    k,b = calib_configs_1 if control < 0 else calib_configs_2
    speed = k * control + b
    return max(speed, 0) if control < 0 else min(speed, 0)

def speed_to_control(speed):
    if abs(speed) < 0.001:  # Deadzone to avoid noise near zero
        return 0
    k,b = calib_configs_1 if speed > 0 else calib_configs_2
    return (speed - b) / k  # Inverse for positive speed

# a = [
#     speed_to_control(0.9),
#     speed_to_control(0.3),
#     speed_to_control(0.03),
#     speed_to_control(0.003),
#     speed_to_control(0.0023389579977694027),
#     speed_to_control(0.0017),
#     speed_to_control(0.0013),
# ]
# print(a)
from dataclasses import dataclass
import numpy as np
from collections import deque
from scipy.optimize import minimize

class BaseCartPoleController:
    def compute_control(self, state: np.ndarray, dt: float) -> float:
        raise NotImplementedError("Control method not implemented.")

    def _wrap_angle(self, angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi

@dataclass
class ControlParams:
    @dataclass
    class PDParams:
        k_theta_p: float = 40000.0
        k_theta_d: float = 1000.0
        k_x_p: float = 300.0
        k_x_d: float = 150.0
        k_x_i: float = 1.0
        k_x_i_dur: float = 2.0  # integral duration in seconds
        switch_angle_deg: float = 45.0  # switching threshold in degrees

    pd: PDParams

class Controller(BaseCartPoleController):
    def __init__(self, method: str = "pd", params: ControlParams = None, max_force=1.0):
        super().__init__()
        self.params = params if params else ControlParams()
        self.method = method.lower()
        self.last_state = 1 
        self.integral_window = deque()
        self.pose_integral = 0.0
        self.max_force = max_force

    def compute_control(self, state) -> float:
        dt = state.dt
        state = np.array([state.x, state.x_dot, state.theta, state.theta_dot])
        state[2] = state[2] + np.pi
        state[3] = state[3] 

        control = 0
        if self.method == "pd":
            control = self._compute_pd_control(state, dt)
        else:
            raise ValueError(f"Unknown control method: {self.method}")
        return np.clip(control, -self.max_force, self.max_force)


    def _compute_pd_control(self, state: np.ndarray, dt: float) -> float:
        x, x_dot, theta, theta_dot = state
        theta = self._wrap_angle(theta)
        p = self.params.pd
        
        sign = 1 if np.abs(theta) <= np.pi / 2 else -1

        # Remove old values beyond the window duration
        total_time = 0.0
        for i in reversed(range(len(self.integral_window))):
            total_time += self.integral_window[i][1]
            if total_time > self.params.pd.k_x_i_dur:
                self.integral_window = deque(list(self.integral_window)[i+1:])
                break

        # Compute the limited integral
        pose_integral = sum(e for e, _ in self.integral_window)

        u_theta = sign * (p.k_theta_p * theta + p.k_theta_d * theta_dot)
        switch_rad = np.radians(p.switch_angle_deg)
        if abs(theta) < switch_rad:
            u_x = -p.k_x_p*x -p.k_x_d*x_dot -p.k_x_i*pose_integral
            self.integral_window.append((-x * dt, dt))
            return u_theta + u_x
        else:
            self.integral_window.append((0, dt))
        return u_theta
import numpy as np

class BaseCartPoleController:
    def compute_control(self, state: np.ndarray) -> float:
        raise NotImplementedError("Control method not implemented.")

    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, +pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi


class CartPoleController(BaseCartPoleController):
    def __init__(self, method='pd'):
        super().__init__()
        self.method = method.lower()

        # PD control gains for [theta, x]
        self.k_theta_p = 80.0
        self.k_theta_d = 15.0
        self.k_x_p = 1.0
        self.k_x_d = 0.5

    def compute_control(self, state: np.ndarray) -> float:
        x, x_dot, theta, theta_dot = state

        if self.method == 'pd':
            return self._pd_control(x, x_dot, theta, theta_dot)
        else:
            raise NotImplementedError(f"Method `{self.method}` is not implemented.")

    def _pd_control(self, x, x_dot, theta, theta_dot):
        theta = self._wrap_angle(theta)
        u_theta = -self.k_theta_p * theta - self.k_theta_d * theta_dot
        u_x = -self.k_x_p * x - self.k_x_d * x_dot
        return u_theta + u_x
    

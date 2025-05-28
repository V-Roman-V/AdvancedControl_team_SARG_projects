import numpy as np
from dataclasses import dataclass

@dataclass
class CartPoleParams:
    m_cart: float = 1.0      # mass of the cart
    m_pole: float = 0.1      # mass of the pole
    l: float = 0.5           # half-length of the pole
    g: float = 9.81          # gravity
    damping: float = 30    # simple friction coefficient
    rotary_damping: float = 0.3  # pole angular damping
    max_force: float = 100.0

class CartPole:
    def __init__(self, init_state: np.ndarray, params: CartPoleParams = None):
        self.state = init_state.astype(np.float64)
        self.params = params if params else CartPoleParams()

    def dynamics(self, state: np.ndarray, force: float) -> np.ndarray:
        """
        State-space form:
            state = [x, x_dot, theta, theta_dot]
            state_dot = [x_dot, x_ddot, theta_dot, theta_ddot]
        """
        x, x_dot, theta, theta_dot = state
        p = self.params

        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        total_mass = p.m_cart + p.m_pole
        pole_mass_length = p.m_pole * p.l

        temp = (force + pole_mass_length * theta_dot**2 * sin_theta - p.damping * x_dot) / total_mass

        theta_acc = (p.g * sin_theta - cos_theta * temp - p.rotary_damping * theta_dot) / \
                    (p.l * (4/3 - p.m_pole * cos_theta**2 / total_mass))

        x_acc = temp - pole_mass_length * theta_acc * cos_theta / total_mass

        return np.array([x_dot, x_acc, theta_dot, theta_acc])

    def update(self, force: float, dt: float):
        force = np.clip(force, -self.params.max_force, self.params.max_force)
        derivs = self.dynamics(self.state, force)
        self.state += derivs * dt
        self.state[2] = (self.state[2] + np.pi) % (2 * np.pi) - np.pi  # wrap theta ∈ [-π, π]

import numpy as np

class CartPole:
    def __init__(self, init_state: np.ndarray):
        self.state = init_state.astype(np.float64)

        # Constants (can be parameterized later)
        self.m_cart = 1.0      # mass of the cart
        self.m_pole = 0.1      # mass of the pole
        self.l = 0.5           # half-length of the pole
        self.g = 9.81          # gravity
        self.damping = 0.01    # simple friction coefficient

    def dynamics(self, state: np.ndarray, force: float) -> np.ndarray:
        x, x_dot, theta, theta_dot = state
        m = self.m_cart
        m_p = self.m_pole
        l = self.l
        g = self.g
        mu = self.damping

        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        total_mass = m + m_p
        pole_mass_length = m_p * l

        temp = (force + pole_mass_length * theta_dot**2 * sin_theta - mu * x_dot) / total_mass
        theta_acc = (g * sin_theta - cos_theta * temp) / (l * (4/3 - m_p * cos_theta**2 / total_mass))
        x_acc = temp - pole_mass_length * theta_acc * cos_theta / total_mass

        return np.array([x_dot, x_acc, theta_dot, theta_acc])

    def update(self, force: float, dt: float):
        derivs = self.dynamics(self.state, force)
        self.state += derivs * dt
        self.state[2] = (self.state[2] + np.pi) % (2 * np.pi) - np.pi  # wrap theta ∈ [-π, π]

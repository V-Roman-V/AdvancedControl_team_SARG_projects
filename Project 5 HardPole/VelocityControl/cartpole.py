import numpy as np
from dataclasses import dataclass

@dataclass
class CartPoleParams:
    M: float = 4.5        # cart mass (kg)
    m: float = 0.300      # pole mass (kg)
    l: float = 0.227      # pole length (m)
    b_c: float = 0.8      # viscous friction (cart)
    f_c: float = 0.9      # Coulomb friction (cart)
    b_p: float = 0.0028   # viscous friction (pole)
    f_p: float = 0.0095   # Coulomb friction (pole)
    K_p: float = 100      # Virtual Force
    max_speed: float = 1.0  # max speed of the cart (m/s)
    g: float = 9.81  # max speed of the cart (m/s)
class CartPole:
    g = 9.81

    def __init__(self, init_state: np.ndarray, params: CartPoleParams = None):
        self.state = init_state.astype(np.float64)

        if params is None:
            params = CartPoleParams()
        self.params_obj = params
        self.params = np.array([
            params.M,       # M: cart mass (kg)
            params.m,       # m: pole mass (kg)
            params.l,       # l: pole length (m)
            params.b_c,     # b_c: viscous friction (cart)
            params.f_c,     # f_c: Coulomb friction (cart)
            params.b_p,     # b_p: viscous friction (pole)
            params.f_p,     # f_p: Coulomb friction (pole)
            params.K_p      # K_p: Virtual Force  
        ])

    def get_dynamic(self, params, state, u_speed=0.0):
        _, x_dot, theta, theta_dot = state
        g = self.g
        M, m, L, b_c, f_c, b_p, f_p, K_pf = params


        # Intermediate terms
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        sgn_x_dot = np.sign(x_dot)
        sgn_theta_dot = np.sign(theta_dot)

        # Friction forces
        F_fric = b_c * x_dot + f_c * sgn_x_dot
        T_fric = b_p * theta_dot + f_p * sgn_theta_dot

        # Virtual Force
        u_f = K_pf * (u_speed - x_dot)

        D = M + m
        mlcos = m * L * cos_theta

        # Compute effective inertia term for theta equation
        alpha = m * L**2 - (mlcos**2) / D

        # Numerator of angular acceleration
        beta = (
            -m * g * L * sin_theta
            - (mlcos / D) * (-M*u_f + F_fric + m * L * theta_dot**2 * sin_theta)
            - T_fric
        )

        # Angular acceleration
        theta_ddot = beta / alpha

        # Linear acceleration
        pole_force = mlcos * theta_ddot - m * L * theta_dot**2 * sin_theta
        x_ddot = u_f + pole_force / D

        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])
    
    @staticmethod
    def dynamics_batch_orig(params: CartPoleParams, states: np.ndarray, u_speeds: np.ndarray) -> np.ndarray:
        """
        Vectorized dynamics for a batch of states and u_speeds.
        states: shape (N, 4) -> [x, x_dot, theta, theta_dot]
        u_speeds: shape (N,)
        Returns: shape (N, 4) -> [x_dot, x_ddot, theta_dot, theta_ddot]
        """
        x_dot = states[:, 1]
        theta = states[:, 2]
        theta_dot = states[:, 3]

        g = 9.81
        np_params = np.array([
            params.M,       # M: cart mass (kg)
            params.m,       # m: pole mass (kg)
            params.l,       # l: pole length (m)
            params.b_c,     # b_c: viscous friction (cart)
            params.f_c,     # f_c: Coulomb friction (cart)
            params.b_p,     # b_p: viscous friction (pole)
            params.f_p,     # f_p: Coulomb friction (pole)
            params.K_p      # K_p: Virtual Force  
        ])
        M, m, L, b_c, f_c, b_p, f_p, K_pf = np_params

        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        sgn_x_dot = np.sign(x_dot)
        sgn_theta_dot = np.sign(theta_dot)

        F_fric = b_c * x_dot + f_c * sgn_x_dot
        T_fric = b_p * theta_dot + f_p * sgn_theta_dot
        u_f = K_pf * (u_speeds - x_dot)

        D = M + m
        mlcos = m * L * cos_theta
        alpha = m * L**2 - (mlcos**2) / D

        beta = (
            -m * g * L * sin_theta
            - (mlcos / D) * (-M * u_f + F_fric + m * L * theta_dot**2 * sin_theta)
            - T_fric
        )

        theta_ddot = beta / alpha
        pole_force = mlcos * theta_ddot - m * L * theta_dot**2 * sin_theta
        x_ddot = u_f + pole_force / D

        derivs = np.stack([x_dot, x_ddot, theta_dot, theta_ddot], axis=-1)
        return derivs
    
    @staticmethod
    def dynamics_batch(params: CartPoleParams, states: np.ndarray, u_speeds: np.ndarray) -> np.ndarray:
        roman_state = states.copy()
        roman_state[:, 0] = -roman_state[:, 0]
        roman_state[:, 1] = -roman_state[:, 1]
        roman_state[:, 2] = np.pi - roman_state[:, 2]
        roman_state[:, 3] = -roman_state[:, 3]
        roman_u_speeds = -u_speeds
        roman_derivs = CartPole.dynamics_batch_orig(params, roman_state, roman_u_speeds)
        artem_derivs = roman_derivs.copy()
        artem_derivs[:, 0] = -artem_derivs[:, 0]
        artem_derivs[:, 1] = -artem_derivs[:, 1]
        artem_derivs[:, 2] = -artem_derivs[:, 2]
        artem_derivs[:, 3] = -artem_derivs[:, 3]
        return artem_derivs
        

    def update(self, speed: float, dt: float):
        speed = np.clip(speed, -self.params_obj.max_speed, self.params_obj.max_speed)

        roman_state = self.state.copy()
        roman_state[0] = - roman_state[0]
        roman_state[1] = - roman_state[1]
        roman_state[2] = np.pi - roman_state[2]
        roman_state[3] = - roman_state[3]

        roman_speed = - speed

        roman_derivs = self.get_dynamic(self.params, roman_state, roman_speed)
        artem_derivs = roman_derivs.copy()
        artem_derivs[0] = - artem_derivs[0]
        artem_derivs[1] = - artem_derivs[1]
        artem_derivs[2] = - artem_derivs[2]
        artem_derivs[3] = - artem_derivs[3]

        self.state += artem_derivs * dt
        self.state[2] = (self.state[2] + np.pi) % (2 * np.pi) - np.pi  # wrap theta ∈ [-π, π]



import numpy as np
from converter import control_to_speed, speed_to_control
from dataclasses import dataclass

@dataclass
class State:
    timestamp: float # in s
    dt: float        # in s
    x: float         # cart position
    x_dot: float     # cart velocity
    theta: float     # pole angle [0:2pi] bottom is 0
    theta_dot: float # pole angular velocity
    old_ctrl: float  # previous control value

    def to_list(self):
        return self.timestamp, self.dt, self.x, self.x_dot, self.theta, self.theta_dot, self.old_ctrl
    
    def _wrap_angle(self, angle):
        '''wrap angle to [0:2pi]'''
        return angle % (2*np.pi)

    def propagate(self, derivatives, dt) -> "State":
        x_dot, x_ddot, theta_dot, theta_ddot = derivatives
        return State(
             timestamp = self.timestamp + dt,
             dt = dt,
             x = self.x + x_dot * dt,
             x_dot = self.x_dot + x_ddot * dt,
             theta = self._wrap_angle(self.theta + theta_dot * dt),
             theta_dot = self.theta_dot + theta_ddot * dt,
             old_ctrl = self.old_ctrl,
        )

    def __str__(self) -> str:
        return f"[{self.timestamp:.4f}] Δt={self.dt:.3f}s | Cart: {self.x:.3f} m @ {self.x_dot:.3f} m/s | "f"Pole: {self.theta:.3f} rad @ {self.theta_dot:.3f} rad/s | u={self.old_ctrl}"

class CartPole:
    g = 9.81

    def __init__(self):
        self.params = np.array([
            4.5,       # M: cart mass (kg)
            0.300,     # m: pole mass (kg)
            0.227,     # l: pole length (m)
            0.8,       # b_c: viscous friction (cart)
            0.9,       # f_c: Coulomb friction (cart)
            0.0028,    # b_p: viscous friction (pole)
            0.0095,    # f_p: Coulomb friction (pole)
            100        # K_p: Virtual Force  
        ])

    def get_dynamic(self, state: State, u_speed=0.0):
        x_dot, theta, theta_dot = state.x_dot, state.theta, state.theta_dot
        g = self.g
        M, m, L, b_c, f_c, b_p, f_p, K_pf = self.params

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
        x_ddot = u_f + pole_force / 100

        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

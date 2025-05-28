from dataclasses import dataclass
import numpy as np
from cartpole import CartPoleParams

class BaseCartPoleController:
    def compute_control(self, state: np.ndarray, dt: float) -> float:
        raise NotImplementedError("Control method not implemented.")

    def _wrap_angle(self, angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi

@dataclass
class ControlParams:
    @dataclass
    class PDParams:
        k_theta_p: float = 2.0
        k_theta_d: float = 5.0
        k_x_p: float = 32.0
        k_x_d: float = 12.0
    @dataclass
    class EnergyParams:
        k_energy: float = 10.0          # gain for energy-based controller
    @dataclass
    class HybridParams:
        switch_angle_deg: float = 15.0  # switching threshold in degrees
    
    pd: PDParams
    energy: EnergyParams
    hybrid: HybridParams

class Controller(BaseCartPoleController):
    def __init__(self, method: str = "hybrid", params: ControlParams = None, cartpole_params: CartPoleParams = None):
        super().__init__()
        self.cartpole_params = cartpole_params if cartpole_params else CartPoleParams()
        self.params = params if params else ControlParams()
        self.method = method.lower()
        self.last_state = 1 

    def compute_control(self, state: np.ndarray, dt: float = 0.02) -> float:
        if self.method == "pd":
            return self._compute_pd_control(state, dt)
        elif self.method == "energy":
            return self._compute_energy_control(state)
        elif self.method == "hybrid":
            return self._compute_hybrid_control(state, dt)
        else:
            raise ValueError(f"Unknown control method: {self.method}")

    def _compute_hybrid_control(self, state: np.ndarray, dt: float) -> float:
        x, x_dot, theta, theta_dot = state
        theta = self._wrap_angle(theta)
        switch_rad = np.radians(self.params.hybrid.switch_angle_deg)
        if abs(theta) < switch_rad:
            if self.last_state == 1:  # Switch from energy to PD
                print("Switching to PD control")
            self.last_state = 0 
            return self._compute_pd_control(state, dt)
        else:
            if self.last_state == 0:  # Switch from PD to energy
                print("Switching to energy control")
            self.last_state = 1
            return self._compute_energy_control(state)

    def _compute_pd_control(self, state: np.ndarray, dt: float) -> float:
        x, x_dot, theta, theta_dot = state
        theta = self._wrap_angle(theta)

        p = self.params.pd
        u_theta = -p.k_theta_p * theta - p.k_theta_d * theta_dot
        u_x = -p.k_x_p * x - p.k_x_d * x_dot
        return u_theta + u_x

    def _compute_energy_control(self, state: np.ndarray) -> float:
        _, _, theta, theta_dot = state
        theta = self._wrap_angle(theta)

        m = self.cartpole_params.m_pole
        l = self.cartpole_params.l
        g = self.cartpole_params.g
        k_energy = self.params.energy.k_energy

        theta_dot = np.clip(theta_dot, -50.0, 50.0)  # prevent overflow

        # Total energy
        KE = 0.5 * m * (l ** 2) * theta_dot ** 2
        PE = -m * g * l * np.cos(theta)
        E = KE + PE
        E_des = -m * g * l

        dE = np.clip(E - E_des, -1000, 1000)
        direction = np.sign(theta_dot * np.cos(theta))
        u = -k_energy * dE * direction

        return u

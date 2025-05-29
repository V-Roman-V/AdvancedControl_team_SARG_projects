from dataclasses import dataclass
import numpy as np
from cartpole import CartPoleParams, CartPole
from collections import deque
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import time

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

    @dataclass
    class HybridParams:
        switch_angle_deg: float = 15.0  # switching threshold in degrees
    @dataclass
    class EnergyParams:
        k_energy: float = 10.0          # gain for energy-based controller
    @dataclass
    class MPCMonteParams:
        horizon_seconds: int = 0.5
        samples: int = 10000
        x_limit: float = 1 # limit for x position
        force_step_part: float = 0.1 # part of max force

        weight_theta: float = 10.0
        weight_theta_dot: float = 0.1
        weight_x: float = 1.0
        weight_x_dot: float = 0.1

    pd: PDParams
    energy: EnergyParams
    hybrid: HybridParams
    mpc_monte: MPCMonteParams

class Controller:
    def __init__(self, method: str = "pd", params: ControlParams = None, cartpole_params: CartPoleParams = None):
        super().__init__()
        self.cartpole_params = cartpole_params if cartpole_params else CartPoleParams()
        self.params = params if params else ControlParams()
        self.method = method.lower()
        self.last_state = 1 
        self.integral_window = deque()
        self.pose_integral = 0.0

    def _wrap_angle(self, angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _wrap_angle_batch(self, angles: np.ndarray) -> np.ndarray:
        return (angles + np.pi) % (2 * np.pi) - np.pi

    def compute_control(self, state: np.ndarray, dt: float = 0.02) -> float:
        control = 0
        if self.method == "pd":
            control = self._compute_pd_control(state, dt)
        elif self.method == "energy":
            control = self._compute_energy_control(state)
        elif self.method == "hybrid":
            control = self._compute_hybrid_control(state, dt)
        elif self.method == "mpc_montecarlo":
            control = self._compute_mpc_montecarlo_control(state, dt)
        else:
            raise ValueError(f"Unknown control method: {self.method}")
        return np.clip(control, -self.cartpole_params.max_force, self.cartpole_params.max_force)

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

    def _compute_energy_control(self, state: np.ndarray) -> float:
        _, _, theta, theta_dot = state
        theta = self._wrap_angle(theta)

        m = self.cartpole_params.m_pole
        l = self.cartpole_params.l
        g = self.cartpole_params.g
        k_energy = self.params.energy.k_energy
        # Compute total mechanical energy relative to upright
        KE = 0.5 * m * (l ** 2) * theta_dot ** 2
        PE = m * g * l * (1 - np.cos(theta))  # 0 when upright
        E = KE + PE
        E_des = 0.0

        dE = E - E_des
        direction = np.sign(theta_dot) if abs(theta_dot) > 1e-3 else 1.0  # prevent zero switching
        u = k_energy * dE * direction
        return u

    def _compute_mpc_montecarlo_control(self, state: np.ndarray,
                                            dt: float = 0.02) -> float:
        p    = self.params.mpc_monte
        N    = int(p.samples)
        H    = int(p.horizon_seconds / dt)       # steps in horizon
        m_f  = self.cartpole_params.max_force
        fspd = m_f * p.force_step_part          # Δu range

        rng  = np.random.default_rng()

        # allocate once
        states = np.repeat(state[None, :], N, axis=0)   # (N,4)
        u0     = rng.normal(0, m_f / 2, N)              # initial force, normal distribution
        u      = u0.copy()

        # all random Δu for the entire horizon except the first step
        du_all = rng.normal(0, fspd / 2, (N, H-1))

        for k in range(H):
            # apply control
            vel   = CartPole.dynamics_batch(self.cartpole_params, states, u)
            states += dt * vel
            states[:, 2] = (states[:, 2] + np.pi) % (2*np.pi) - np.pi  # wrap θ
            states[np.abs(states[:, 0]) > p.x_limit][:,0] = np.inf  # ruine the pose

            if k < H-1:       # update force for next step
                u += du_all[:, k]
                np.clip(u, -m_f, m_f, out=u)

        # cost
        x, x_dot, th, th_dot = states.T
        th = (th + np.pi) % (2*np.pi) - np.pi
        cost = (p.weight_theta*th**2 + p.weight_theta_dot*th_dot**2 +
                p.weight_x*x**2     + p.weight_x_dot*x_dot**2)
        cost[np.abs(x) > p.x_limit] = np.inf

        best = cost.argmin()
        return float(u0[best])            # u0 is the action to apply
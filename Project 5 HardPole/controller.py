from dataclasses import dataclass
import numpy as np
from cartpole import CartPoleParams, CartPole
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

    @dataclass
    class HybridParams:
        switch_angle_deg: float = 15.0  # switching threshold in degrees
    @dataclass
    class EnergyParams:
        k_energy: float = 10.0          # gain for energy-based controller
    @dataclass
    class MPCParams:
        horizon_steps: int = 20
        candidate_force_count: int = 9
        weight_theta: float = 10.0
        weight_theta_dot: float = 0.1
        weight_x: float = 1.0
        weight_x_dot: float = 0.1

    pd: PDParams
    energy: EnergyParams
    hybrid: HybridParams
    mpc: MPCParams

class Controller(BaseCartPoleController):
    def __init__(self, method: str = "pd", params: ControlParams = None, cartpole_params: CartPoleParams = None):
        super().__init__()
        self.cartpole_params = cartpole_params if cartpole_params else CartPoleParams()
        self.params = params if params else ControlParams()
        self.method = method.lower()
        self.last_state = 1 
        self.integral_window = deque()
        self.pose_integral = 0.0

    def compute_control(self, state: np.ndarray, dt: float = 0.02) -> float:
        control = 0
        if self.method == "pd":
            control = self._compute_pd_control(state, dt)
        elif self.method == "energy":
            control = self._compute_energy_control(state)
        elif self.method == "hybrid":
            control = self._compute_hybrid_control(state, dt)
        elif self.method == "mpc_simple":
            control = self._compute_mpc_simple_control(state, dt)
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

    def _compute_mpc_simple_control(self, state: np.ndarray, dt: float = 0.02) -> float:
        p = self.params.mpc
        N = p.horizon_steps
        candidate_us = np.linspace(-self.cartpole_params.max_force,
                                self.cartpole_params.max_force,
                                p.candidate_force_count)
        best_cost = float('inf')
        best_u = 0.0

        def cost_fn(trajectory):
            cost = 0.0
            for s in trajectory:
                x, x_dot, theta, theta_dot = s
                theta = self._wrap_angle(theta)
                cost += (
                    p.weight_theta * theta**2 +
                    p.weight_theta_dot * theta_dot**2 +
                    p.weight_x * x**2 +
                    p.weight_x_dot * x_dot**2
                )
            return cost

        for u in candidate_us:
            traj = [state.copy()]
            s = state.copy()
            for _ in range(N):
                s_dot = CartPole.dynamics(self.cartpole_params, s, u)
                s = s + s_dot * dt
                s[2] = self._wrap_angle(s[2])
                traj.append(s.copy())
            cost = cost_fn(traj)
            if cost < best_cost:
                best_cost = cost
                best_u = u

        return best_u
    
    def _compute_mpc_control(self, state: np.ndarray, dt: float = 0.02) -> float:
        p = self.params.mpc
        max_f = self.cartpole_params.max_force
        horizon = p.horizon_steps

        def trajectory_cost(u_scalar):
            u = float(u_scalar[0])
            s = state.copy()
            total_cost = 0.0
            for _ in range(horizon):
                s_dot = CartPole(state=s, params=self.cartpole_params).dynamics(s, u)
                s = s + dt * s_dot
                s[2] = self._wrap_angle(s[2])
                x, x_dot, theta, theta_dot = s
                total_cost += (
                    p.weight_theta * theta**2 +
                    p.weight_theta_dot * theta_dot**2 +
                    p.weight_x * x**2 +
                    p.weight_x_dot * x_dot**2
                )
            return total_cost

        res = minimize(
            trajectory_cost,
            x0=[0.0],
            method='L-BFGS-B',
            bounds=[(-max_f, max_f)],
            options={'maxiter': 20}
        )

        optimal_u = float(res.x[0])
        return np.clip(optimal_u, -max_f, max_f)

import numpy as np
from boat import BoatParameters

class Controller:
    def __init__(self, boat_params: BoatParameters):
        """
        Initialize controller with physical parameters
        """
        self.boat_params = boat_params

    def compute_control(self, state: np.ndarray, state_des: np.ndarray) -> np.ndarray:
        """
        Compute control inputs based on the current state and desired state.

        Args:
            state: Current state [x, y, psi, Vx, Vy, omega, adapt_param1, adapt_param2]
            state_des: Desired state [x_d, y_d, 0, 0, 0, 0, 0, 0] - Only position is important
        """
        raise NotImplementedError("Dynamics method not implemented.")

    def _wrap_angle(self, angle):
        """Wraps angle to [-pi, +pi] using atan2-style wrapping."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

class DifferentialController(Controller):
    """
    Differential thrust controller for a boat.
    """
    def __init__(self, boat_params: BoatParameters, control_limit=10.0,):
        super().__init__(boat_params)
        self.control_limit = control_limit
        
        m = self.boat_params.mass
        i = self.boat_params.inertia
        # Controller gains and adaptation rates
        self.k1 = 200 / m # Transitional 
        self.k2 = 100 / i # Rotational
        self.k3 = 130 / m # Transitional = 2*sqrt(k1)
        self.k4 = 200 / i # Rotational = 2*sqrt(k2)
        self.gamma1 = 0.005
        self.gamma2 = 0.003

    def compute_control(self, state: np.ndarray, state_des: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Energy-based control computation

        Returns:
            np.ndarray: Thruster commands [u1, u2]
            np.ndarray: Adaptation derivatives [dAp_1, dAp_2]
        """
        # --- Step 1: Extract states ---
        x, y, psi, Vx, Vy, omega, adapt_param1, adapt_param2 = state
        omega = -omega
        x_d, y_d = state_des[:2]

        # --- Step 2: Error calculation ---
        e_x = x_d - x
        e_y = y_d - y
        psi_d = np.arctan2(e_y, e_x)
        e_psi = self._wrap_angle(psi - psi_d)

        # Forward error
        e_f = (np.cos(e_psi) * e_x + np.sin(e_psi) * e_y) * np.sign(x)

        # Desired virtual velocities
        Vx_des = -self.k1 * e_f
        omega_des = self.k2 * e_psi

        # Velocity tracking errors
        e_bar_x = Vx - Vx_des
        e_bar_omega = omega - omega_des
        # --- Step 3: Compute control force/moment ---
        Fx = -self.boat_params.mass * self.k3 * e_bar_x + self.boat_params.mass * adapt_param1
        M = -self.boat_params.inertia * self.k4 * e_bar_omega + self.boat_params.inertia * adapt_param2
        # print(f"Fx = {Fx:.3f} A={self.boat_params.mass * self.k3 * e_bar_x:.3f} fix={self.boat_params.mass * adapt_param1:.3f} | M = {M:.3f} fix={self.boat_params.inertia * adapt_param2:.3f}")

        # --- Step 4: Map to differential thrusts ---

        u1 = 0.5 * (Fx + M / self.boat_params.L)
        u2 = 0.5 * (Fx - M / self.boat_params.L)

        if u1 < 0 and u2 < 0:
            Fx = np.clip(Fx, - 0.8 * self.control_limit, 0.8 * self.control_limit)
        elif u1 > self.control_limit and u2 > self.control_limit:
            Fx = np.clip(Fx, - 0.8 * self.control_limit, 0.8 * self.control_limit)
        u1 = 0.5 * (Fx + M / self.boat_params.L)
        u2 = 0.5 * (Fx - M / self.boat_params.L)

        # --- Step 5: Thrust direction fix (turnaround logic) ---
        if u1 < 0 and u2 < 0:
            if abs(u1) > abs(u2):
                u1 = 0.5 * self.control_limit
                u2 = 0
            else:
                u2 = 0.5 * self.control_limit
                u1 = 0

        # --- Step 6: Apply joint saturation with ratio preservation ---
        u1_raw = u1
        u2_raw = u2
        max_thrust = self.control_limit

        max_val = max(abs(u1_raw), abs(u2_raw))
        if max_val > max_thrust:
            scale = max_thrust / max_val
            u1 = scale * u1_raw
            u2 = scale * u2_raw
        else:
            u1 = u1_raw
            u2 = u2_raw

        # Final safety clip to [0, max_thrust]
        u1 = np.clip(u1, 0, max_thrust)
        u2 = np.clip(u2, 0, max_thrust)

        # --- Step 7: Adaptation law ---
        coeff = 1
        if abs(e_f) < 0.1:
            coeff = 2
        dAp_1 = -self.gamma1 * e_bar_x * coeff
        dAp_2 = -self.gamma2 * e_bar_omega

        # Return thruster commands and Adaptation derivatives
        return np.array([u1, u2]), np.array([dAp_1, dAp_2])

class SteeringController(Controller):
    """
    Steering thrust controller for a boat.
    """
    def __init__(self, boat_params: BoatParameters, control_limit: list = (10.0, np.pi/2)):
        super().__init__(boat_params)
        self.control_limit = control_limit

        m = self.boat_params.mass
        # Controller gains and adaptation rates
        self.k1 = 1.21 / m # Transitional 
        self.k2 = 0.64 # Rotational
        self.k3 = 2.2 / m # Transitional = 2*sqrt(k1)
        self.k4 = 1.6 # Rotational = 2*sqrt(k2)
        self.gamma1 = 0 #0.15
        self.gamma2 = 0 #0.003

    def compute_control(self, state: np.ndarray, state_des: np.ndarray) -> np.ndarray:
        """
        Energy-based control computation with clear steps
            
        Returns:
            np.ndarray: Thruster commands [u_forward, u_steer]
            np.ndarray: Adaptation derivatives [dAp_1, dAp_2]
        """
        # --- Step 1: Extract states ---
        x, y, psi, Vx, Vy, omega, adapt_param1, adapt_param2 = state
        x_d, y_d = state_des[:2]

        # --- Step 2: Error calculation ---
        e_x = x_d - x
        e_y = y_d - y
        psi_d = np.arctan2(e_y, e_x)
        e_psi = self._wrap_angle(psi - psi_d)
        if abs(e_psi) > np.pi/2:
            e_psi += np.pi

        # Forward error
        e_f = np.cos(e_psi) * e_x + np.sin(e_psi) * e_y

        # Desired virtual velocities
        Vx_des = self.k1 * e_f
        omega_des = -self.k2 * e_psi

        # Velocity tracking errors
        e_bar_x = Vx - Vx_des
        e_bar_omega = omega - omega_des

        # Regressor vectors
        phi_x = np.array([Vx])
        phi_psi = np.array([omega])

        # --- Step 3: Compute control ---
        Fx = -self.boat_params.mass * self.k3 * e_bar_x #+ self.boat_params.mass * adapt_param1 * phi_x[0]
        M = -self.boat_params.inertia * self.k4 * e_bar_omega #+ self.boat_params.inertia * adapt_param2 * phi_psi[0]

        # print(f"{e_f = } {e_bar_x = } {Fx = } | {e_psi = } {e_bar_omega = } {M = }")
        # --- Step 4: Convert to polar thruster commands ---
        u_phi = np.arctan2(M / self.boat_params.L, Fx)
        u_f = np.sqrt(Fx**2 + (M / self.boat_params.L)**2)

        # --- Step 5: Turnaround logic ---
        if u_f < 0:
            u_f = 0.5 * self.control_limit[0]
            u_phi = np.sign(self._wrap_angle(u_phi)) * self.control_limit[1]

        # --- Step 6: Apply saturation ---
        u_f = np.clip(u_f, 0, self.control_limit[0])
        u_phi = np.clip(self._wrap_angle(u_phi), -self.control_limit[1], self.control_limit[1])

        # --- Step 7: Adaptation law ---
        dAp_1 = self.gamma1 * phi_x[0] * e_bar_x
        dAp_2 = self.gamma2 * phi_psi[0] * e_bar_omega

        # Return thruster commands and Adaptation derivatives
        return np.array([u_f, u_phi]), np.array([dAp_1, dAp_2])

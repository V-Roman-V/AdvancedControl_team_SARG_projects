import numpy as np
from boat import BoatParameters

class Controller:
    def __init__(self, boat_params: BoatParameters):
        """
        Initialize controller with physical parameters
        """
        self.boat_params = boat_params

    def compute_control(self, state: np.array, state_des: np.array) -> np.ndarray:
        """
        Compute control inputs based on the current state and desired state.

        Args:
            state: Current state [x, y, psi, Vx, Vy, omega, Wx, Wy]
            state_des: Desired state [x_d, y_d, psi_d, Vx_d, Vy_d, omega_d]
        """
        raise NotImplementedError("Dynamics method not implemented.")


class DifferentialController(Controller):
    """
    Differential thrust controller for a boat.
    """
    def __init__(self, boat_params: BoatParameters, control_limit=10.0,):
        super().__init__(boat_params)
        self.control_limit = control_limit
        
        # Controller gains - tune these for performance
        self.k_0 = 2.5  # Position error gain
        self.k_1 = 6  # Velocity-position error gain
        self.k_2 = 1.4  # Heading error gain
        self.k_w = 1/5 * self.boat_params.mass

        self.gamma_wind = 0.3  # wind coefficient

    def compute_control(self, state: np.array, state_des: np.array) -> tuple[np.ndarray, np.ndarray]:
        """
        Energy-based control computation

        Returns:
            np.ndarray: Thruster commands [u1, u2]
            np.ndarray: Wind estimation derivatives [dWx_est, dWy_est]
        """
        # --- Step 1: Extract states ---
        x, y, psi, Vx, Vy, omega, Wx_est, Wy_est = state
        x_d, y_d = state_des[:2]
        
        # --- Step 2: Calculating errors in the body-fixed frame ---
        global_error = np.array([x_d - x, y_d - y])
        R = np.array([[np.cos(psi), np.sin(psi)],
                      [-np.sin(psi), np.cos(psi)]])
        x_e, y_e = R @ global_error  # boat_frame_error
        psi_e = np.arctan2(y_e, x_e)

        # --- Step 3: Wind transformation ---
        Vw_x_body = Wx_est * np.cos(psi) + Wy_est * np.sin(psi)  # Wind velocity in body frame
        Vw_y_body = -Wx_est * np.sin(psi) + Wy_est * np.cos(psi)
        
        # print(f"wind D: {Vw_x_body}, {Vw_y_body}")

        # --- Step 4: Compute control ---
        u1 = self.k_0 * (x_e) - self.k_1 * (x_e * (Vx-Vw_x_body) + y_e * (Vy-Vw_y_body) - omega) - self.k_2 * psi_e - self.k_w * Vw_x_body + 2 * Vx*self.boat_params.damping[0]
        u2 = self.k_0 * (x_e) - self.k_1 * (x_e * (Vx-Vw_x_body) + y_e * (Vy-Vw_y_body) + omega) + self.k_2 * psi_e - self.k_w * Vw_x_body + 2 * Vx*self.boat_params.damping[0]
        
        # turnaround
        if u1 < 0 and u2 < 0:
            if abs(u1) > abs(u2):
                u1 = 1/2 * self.control_limit
                u2 = 0
            else:
                u2 = 0
                u1 = 1/2 * self.control_limit

        # --- Step 4: Apply saturation ---
        u1 = np.clip(u1, 0, self.control_limit)
        u2 = np.clip(u2, 0, self.control_limit)

        # --- Step 5: Wind Estimation: update wind derivative ---
        # We estimate the wind using the difference between the observed and expected velocities in global frame
        Vx_global = Vx * np.cos(psi) - Vy * np.sin(psi)
        Vy_global = Vx * np.sin(psi) + Vy * np.cos(psi)
        
        # Wind estimation adaptation law (simple proportional update rule)
        dWx = self.gamma_wind * (-Vx_global - Wx_est)
        dWy = self.gamma_wind * (-Vy_global - Wy_est)

        # Return thruster commands and wind estimation derivatives
        return np.array([u1, u2]), np.array([dWx, dWy])

class SteeringController(Controller):
    """
    Steering thrust controller for a boat.
    """
    def __init__(self, boat_params: BoatParameters, control_limit: list = (10.0, np.pi/2)):
        super().__init__(boat_params)
        self.control_limit = control_limit
        
        # Controller gains - tune these for performance
        self.k_0 = 1  # Position error gain
        self.k_1 = 3  # Velocity-position error gain
        self.k_2 = 0.1  # Heading error gain
        self.k_w = 2.4/5 * self.boat_params.mass

        self.gamma_wind = 0.3  # wind coefficient

    def compute_control(self, state: np.array, state_des: np.array) -> np.ndarray:
        """
        Energy-based control computation with clear steps
            
        Returns:
            np.ndarray: Thruster commands [u_forward, u_steer]
        """
        # --- Step 1: Extract states ---
        x, y, psi, Vx, Vy, omega, Wx_est, Wy_est = state
        x_d, y_d = state_des[:2]
        
        # --- Step 2: Calculating errors in the body-fixed frame ---
        global_error = np.array([x_d - x, y_d - y])
        R = np.array([[np.cos(psi), np.sin(psi)],
                      [-np.sin(psi), np.cos(psi)]])
        x_e, y_e = R @ global_error  # boat_frame_error
        psi_e = np.arctan2(y_e, x_e)

        # --- Step 3: Wind transformation ---
        Vw_x_body = Wx_est * np.cos(psi) + Wy_est * np.sin(psi)  # Wind velocity in body frame
        Vw_y_body = -Wx_est * np.sin(psi) + Wy_est * np.cos(psi)
        # print(f"wind S : {Vw_x_body}, {Vw_y_body}")

        # --- Step 4: Compute control ---
        uf = self.k_0 * (x_e**2 + y_e**2) - self.k_1 * (x_e * (Vx-Vw_x_body) + y_e * (Vy-Vw_y_body)) - self.k_w * Vw_x_body + Vx*self.boat_params.damping[0]
        us = self.k_2 * psi_e
        
        # turnaround
        if uf < 0:
            uf = 1/2 * self.control_limit[0]
            us = np.sign(self._wrap_angle(us)) * self.control_limit[1]

        # --- Step 5: Apply saturation ---
        uf = np.clip(uf, 0, self.control_limit[0])
        us = np.clip(self._wrap_angle(us), -self.control_limit[1], self.control_limit[1])

        # --- Step 6: Wind Estimation: update wind derivative ---
        # We estimate the wind using the difference between the observed and expected velocities in global frame
        Vx_global = Vx * np.cos(psi) - Vy * np.sin(psi)
        Vy_global = Vx * np.sin(psi) + Vy * np.cos(psi)
        
        # Wind estimation adaptation law (simple proportional update rule)
        dWx = self.gamma_wind * (-Vx_global - Wx_est)
        dWy = self.gamma_wind * (-Vy_global - Wy_est)

        return np.array([uf, us]), np.array([dWx, dWy])
    
    def _wrap_angle(self,angle):
        """
        Wraps the given angle to the range [-pi, +pi].

        :param angle: The angle (in rad) to wrap (can be unbounded).
        :return: The wrapped angle (guaranteed to in [-pi, +pi]).
        """
        pi2 = 2 * np.pi

        while angle < -np.pi:
            angle += pi2

        while angle >= np.pi:
            angle -= pi2

        return angle

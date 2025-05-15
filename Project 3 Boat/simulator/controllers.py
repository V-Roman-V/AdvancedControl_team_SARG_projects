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
            state: Current state [x, y, psi, Vx, Vy, omega, adapt_param1, adapt_param2]
            state_des: Desired state [x_d, y_d, 0, 0, 0, 0, 0, 0] - Only position is important
        """
        raise NotImplementedError("Dynamics method not implemented.")


class DifferentialController(Controller):
    """
    Differential thrust controller for a boat.
    """
    def __init__(self, boat_params: BoatParameters, control_limit=10.0,):
        super().__init__(boat_params)
        self.control_limit = control_limit
        
        # Controller gains and gammas - tune these for performance
        # TODO
        self.k_0 = ...
        self.k_1 = ...
        self.k_3 = ...
        self.k_4 = ...
        self.gamma_1 = ... 
        self.gamma_2 = ...

    def compute_control(self, state: np.array, state_des: np.array) -> tuple[np.ndarray, np.ndarray]:
        """
        Energy-based control computation

        Returns:
            np.ndarray: Thruster commands [u1, u2]
            np.ndarray: Adaptation derivatives [dAp_1, dAp_2]
        """
        # --- Step 1: Extract states ---
        x, y, psi, Vx, Vy, omega, adapt_param1, adapt_param2 = state
        x_d, y_d = state_des[:2]
        
        # --- Step 2: Calculating errors in the body-fixed frame ---
        global_error = np.array([x_d - x, y_d - y])
        R = np.array([[np.cos(psi), np.sin(psi)],
                      [-np.sin(psi), np.cos(psi)]])
        x_e, y_e = R @ global_error  # boat_frame_error
        psi_e = np.arctan2(y_e, x_e)

        # --- Step 3: Compute control ---
        # TODO
        u1 = ...
        u2 = ...

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

        # --- Step 5: Calculate Adaptation derivatives ---
        # TODO
        dAp_1 = ...
        dAp_2 = ...

        # Return thruster commands and Adaptation derivatives
        return np.array([u1, u2]), np.array([dAp_1, dAp_2])

class SteeringController(Controller):
    """
    Steering thrust controller for a boat.
    """
    def __init__(self, boat_params: BoatParameters, control_limit: list = (10.0, np.pi/2)):
        super().__init__(boat_params)
        self.control_limit = control_limit
        
        # Controller gains and gammas - tune these for performance
        # TODO
        self.k_0 = ...
        self.k_1 = ...
        self.k_3 = ...
        self.k_4 = ...
        self.gamma_1 = ... 
        self.gamma_2 = ...

    def compute_control(self, state: np.array, state_des: np.array) -> np.ndarray:
        """
        Energy-based control computation with clear steps
            
        Returns:
            np.ndarray: Thruster commands [u_forward, u_steer]
            np.ndarray: Adaptation derivatives [dAp_1, dAp_2]
        """
        # --- Step 1: Extract states ---
        x, y, psi, Vx, Vy, omega, adapt_param1, adapt_param2 = state
        x_d, y_d = state_des[:2]
        
        # --- Step 2: Calculating errors in the body-fixed frame ---
        global_error = np.array([x_d - x, y_d - y])
        R = np.array([[np.cos(psi), np.sin(psi)],
                      [-np.sin(psi), np.cos(psi)]])
        x_e, y_e = R @ global_error  # boat_frame_error
        psi_e = np.arctan2(y_e, x_e)

        # --- Step 3: Compute control ---
        # TODO
        uf = ...
        us = ...
        
        # turnaround
        if uf < 0:
            uf = 1/2 * self.control_limit[0]
            us = np.sign(self._wrap_angle(us)) * self.control_limit[1]

        # --- Step 4: Apply saturation ---
        uf = np.clip(uf, 0, self.control_limit[0])
        us = np.clip(self._wrap_angle(us), -self.control_limit[1], self.control_limit[1])

        # --- Step 5: Calculate Adaptation derivatives ---
        # TODO
        dAp_1 = ...
        dAp_2 = ...

        # Return thruster commands and Adaptation derivatives
        return np.array([uf, us]), np.array([dAp_1, dAp_2])
    
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

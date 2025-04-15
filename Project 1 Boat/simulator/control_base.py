import numpy as np

class Controller:
    def __init__(self, mass, inertia, damping, control_limit=10.0):
        """
        Initialize controller with physical parameters
        
        Args:
            mass: Boat mass (kg)
            inertia: Yaw inertia (kg·m²)
            damping: List of damping coefficients [D_x, D_y, D_psi]
            control_limit: Maximum thruster force (N)
        """
        self.control_limit = control_limit
        self.mass = mass
        self.inertia = inertia
        self.D_x, self.D_y, self.D_psi = damping

    def compute_control(self, state: np.ndarray, state_des: np.ndarray):
        """
        Compute thruster commands
        
        Args:
            state: Current state [x, y, psi, Vx, Vy, omega]
            state_des: Desired state [x_d, y_d, psi_d, Vx_d, Vy_d, omega_d]
            
        Returns:
            np.ndarray: Thruster commands [u1, u2] - Left and Right thruster
        """
        # Unpack current state
        x, y, psi, Vx, Vy, omega = state
        # Unpack desired state
        x_d, y_d, psi_d = state_des[:3]
        
        # TODO implement logic 
        u1 = 10
        u2 = 15

        # Apply saturation constraints
        u1 = np.clip(u1, 0, self.control_limit)
        u2 = np.clip(u2, 0, self.control_limit)
        
        return np.array([u1, u2])
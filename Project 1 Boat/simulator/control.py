import numpy as np

class EnergyBasedController:
    """
    Energy-based controller for boat trajectory tracking.
    Ensures non-negative surge velocity (no reversing) and aligns the boat towards the target position.
    Uses body-frame position errors for surge and yaw control.
    """
    
    def __init__(self, mass, inertia, damping, 
                 control_limit=10.0,
                 k_p_surge=2.0, 
                 k_v=5.0,
                 Vx_max=2.0,
                 k_p_yaw=0.8,
                 k_omega=3.0):
        """
        Initialize the energy-based controller.

        Args:
            mass (float): Boat mass (kg).
            inertia (float): Yaw inertia (kg·m²).
            damping (list): Damping coefficients [D_x, D_y, D_psi].
            control_limit (float): Maximum thrust force per thruster (N).
            k_p_surge (float): Surge position error gain.
            k_v (float): Surge velocity tracking gain.
            Vx_max (float): Maximum allowed surge velocity (m/s).
            k_p_yaw (float): Yaw position error gain.
            k_omega (float): Angular velocity tracking gain.
        """
        self.mass = mass
        self.inertia = inertia
        self.D_x, self.D_y, self.D_psi = damping
        self.control_limit = control_limit

        # Control gains
        self.k_p_surge = k_p_surge
        self.k_v = k_v
        self.Vx_max = Vx_max
        self.k_p_yaw = k_p_yaw
        self.k_omega = k_omega

    def compute_control(self, state: np.ndarray, state_des: np.ndarray):
        """
        Compute thruster commands using energy-based control law.

        Args:
            state (np.ndarray): Current state [x, y, psi, Vx, Vy, omega].
            state_des (np.ndarray): Desired state [x_d, y_d, _, _, _, _] (rotation ignored).

        Returns:
            np.ndarray: Thruster commands [u1, u2] clamped within control limits.
        """
        # Extract current state components
        x, y, psi, Vx, Vy, omega = state
        x_d, y_d = state_des[0], state_des[1]

        # Position error in global frame
        e_x = x_d - x
        e_y = y_d - y

        # Convert position error to body frame
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        e_x_body = e_x * cos_psi + e_y * sin_psi  # Surge direction error
        e_y_body = -e_x * sin_psi + e_y * cos_psi  # Sway direction error

        # ---- Surge Control ----
        # Desired surge velocity based on body-frame position error
        Vx_d = self.k_p_surge * e_x_body
        Vx_d = np.clip(Vx_d, 0.0, self.Vx_max)  # Ensure non-negative and within limit

        # Surge force to track desired velocity (prevents reverse thrust)
        F_total = self.mass * (self.k_v * (Vx_d - Vx) + self.D_x * Vx)
        F_total = np.maximum(F_total, 0.0)  # Prevent reverse thrust

        # ---- Yaw Control ----
        # Desired angular velocity to reduce sway error
        omega_d = self.k_p_yaw * e_y_body
        
        # Torque to track desired angular velocity
        M_total = self.inertia * (self.k_omega * (omega_d - omega) + self.D_psi * omega)

        # ---- Thrust Allocation ----
        # Convert F_total and M_total to individual thruster commands
        u1 = (F_total - M_total) / 2.0
        u2 = (F_total + M_total) / 2.0

        # Apply thruster saturation limits
        u1 = np.clip(u1, -self.control_limit, self.control_limit)
        u2 = np.clip(u2, -self.control_limit, self.control_limit)

        return np.array([u1, u2])
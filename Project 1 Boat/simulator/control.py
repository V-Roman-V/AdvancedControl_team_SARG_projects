import numpy as np

class Controller:
    """
    Lyapunov-based nonlinear controller for boat trajectory tracking
    Implements surge velocity control, heading control, and thrust allocation
    """
    
    def __init__(self, mass, inertia, damping, 
                 control_limit=10.0,
                 k_p_pos=0.5, 
                 k_p_v=2.0,
                 Vx_max=2.0,
                 k_p_psi=0.5,
                 k_p_omega=2.0,
                 omega_max=1.0):
        """
        Initialize controller with physical parameters and gains
        
        Args:
            mass: Boat mass (kg)
            inertia: Yaw inertia (kg·m²)
            damping: List of damping coefficients [D_x, D_y, D_psi]
            control_limit: Maximum thruster force (N)
            k_p_pos: Position error to velocity gain
            k_p_v: Velocity tracking gain
            Vx_max: Maximum surge velocity (m/s)
            k_p_psi: Heading error to angular velocity gain
            k_p_omega: Angular velocity tracking gain
            omega_max: Maximum angular velocity (rad/s)
        """
        self.control_limit = control_limit
        self.mass = mass
        self.inertia = inertia
        self.D_x, self.D_y, self.D_psi = damping
        
        # Control gains
        self.k_p_pos = k_p_pos
        self.k_p_v = k_p_v
        self.Vx_max = Vx_max
        self.k_p_psi = k_p_psi
        self.k_p_omega = k_p_omega
        self.omega_max = omega_max

    def compute_control(self, state: np.ndarray, state_des: np.ndarray):
        """
        Compute thruster commands using Lyapunov-based control law
        
        Args:
            state: Current state [x, y, psi, Vx, Vy, omega]
            state_des: Desired state [x_d, y_d, psi_d, Vx_d, Vy_d, omega_d]
            
        Returns:
            np.ndarray: Thruster commands [u1, u2]
        """
        # Unpack current state
        x, y, psi, Vx, Vy, omega = state
        # Unpack desired state
        x_d, y_d, psi_d = state_des[:3]
        
        # Position error components
        e_x = x_d - x
        e_y = y_d - y
        
        # ---- Surge control ----
        # Distance to target
        distance_error = np.sqrt(e_x**2 + e_y**2)
        # Desired surge velocity (vector field)
        Vx_d = self.k_p_pos * distance_error
        Vx_d = np.clip(Vx_d, -self.Vx_max, self.Vx_max)
        # Surge control force
        F = self.mass * (self.D_x * Vx + self.k_p_v * (Vx_d - Vx))
        
        # ---- Heading control ----
        # Wrapped heading error [-π, π]
        e_psi = (psi_d - psi + np.pi) % (2 * np.pi) - np.pi
        # Desired angular velocity
        omega_d = self.k_p_psi * e_psi
        omega_d = np.clip(omega_d, -self.omega_max, self.omega_max)
        # Torque control
        M = self.inertia * (self.D_psi * omega + self.k_p_omega * (omega_d - omega))

        # ---- Thrust allocation ----
        scale = 0.01
        u1 = scale * (F - M) / 2  # Left thruster
        u2 = scale * (F + M) / 2  # Right thruster
        
        # Apply saturation constraints
        u1 = np.clip(u1, -self.control_limit, self.control_limit)
        u2 = np.clip(u2, -self.control_limit, self.control_limit)
        
        return np.array([u1, u2])
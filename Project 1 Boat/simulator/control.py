import numpy as np

class Controller:
    def __init__(self, mass, inertia, damping, control_limit=10.0, dt=0.01):
        """
        Initialize controller with physical parameters
        
        Args:
            mass: Boat mass (kg)
            inertia: Yaw inertia (kg·m²)
            damping: List of damping coefficients [D_x, D_y, D_psi]
            control_limit: Maximum thruster force (N)
            dt: Time step for discrete control
        """
        self.control_limit = control_limit
        self.mass = mass
        self.inertia = inertia
        self.D_x, self.D_y, self.D_psi = damping
        self.dt = dt
        
        # Controller gains - tune these for performance
        self.k_p = 7.0    # Position error gain
        self.k_v = .05     # Velocity damping gain
        self.k_omega = 2.0 # Angular velocity gain
        self.k_heading = 1  # Heading alignment gain
        
        # Energy terms for monitoring (optional)
        self.total_energy = 0
        self.energy_derivative = 0

    def compute_control(self, state: np.ndarray, state_des: np.ndarray):
        """
        Energy-based control computation with clear steps
        
        Args:
            state: Current state [x, y, psi, Vx, Vy, omega]
            state_des: Desired state [x_d, y_d, psi_d, Vx_d, Vy_d, omega_d]
            
        Returns:
            np.ndarray: Thruster commands [u1, u2]
        """
        # --- Step 1: Extract states ---
        x, y, psi, Vx, Vy, omega = state
        x_d, y_d, _ = state_des[:3]  # Only care about position
        
        # --- Step 2: Calculate position error ---
        # Global frame error
        global_error = np.array([x_d - x, y_d - y])
        
        # Convert to boat frame using rotation matrix
        R = np.array([[np.cos(psi), np.sin(psi)],
                      [-np.sin(psi), np.cos(psi)]])
        boat_frame_error = R @ global_error
        x_error_b, y_error_b = boat_frame_error
        
        # --- Step 3: Compute energy terms ---
        # Kinetic energy terms
        kinetic_trans = 0.5 * self.mass * (Vx**2 + Vy**2)
        kinetic_rot = 0.5 * self.inertia * omega**2
        
        # Potential energy (virtual spring)
        potential = 0.5 * self.k_p * (global_error[0]**2 + global_error[1]**2)
        
        # Total energy (for monitoring)
        self.total_energy = kinetic_trans + kinetic_rot + potential
        
        # --- Step 4: Compute desired forces ---
        # Longitudinal force (dissipate Vx while driving x error to zero)
        Fx_des = (self.k_p * x_error_b            # Position correction
                 - self.k_v * Vx                  # Velocity damping
                 + self.D_x * Vx)                 # Compensate natural damping
        
        # Lateral force cannot be directly controlled, so we use heading alignment
        desired_omega = self.k_heading * np.arctan2(y_error_b, x_error_b)
            
        omega_error = desired_omega - omega
        
        # Desired torque (dissipate omega while aligning to path)
        tau_des = (self.k_omega * omega_error     # Angular velocity correction
                 + self.D_psi * omega)            # Compensate natural damping
        
        # --- Step 5: Convert to thruster commands ---
        # Solve u1 + u2 = Fx_des
        #       u2 - u1 = tau_des * lever_arm (assuming 1m for simplicity)
        u1 = 0.5 * (Fx_des - tau_des)
        u2 = 0.5 * (Fx_des + tau_des)
        
        # --- Step 6: Apply saturation ---
        u1 = np.clip(u1, 0, self.control_limit)
        u2 = np.clip(u2, 0, self.control_limit)
        
        # --- (Optional) Energy derivative calculation ---
        # For monitoring stability - not needed for control
        if hasattr(self, 'prev_energy'):
            self.energy_derivative = (self.total_energy - self.prev_energy) / self.dt
        self.prev_energy = self.total_energy
        
        return np.array([u1, u2]), self.total_energy, self.energy_derivative
import numpy as np

class Boat:
    """
    Boat class models the vessel's dynamics.

    It defines the vessel's state and updates its state over time based on the control inputs.
    The dynamics include the kinematics of the vessel and simple force-based equations of motion.
    """

    def __init__(self, initial_state: list, mass: float, inertia: float, damping: list):
        """
        Initializes the Boat object with initial conditions and system parameters.

        Args:
            initial_state (list): Initial state of the vessel [x, y, psi, Vx, Vy, omega].
            mass (float): Mass of the vessel (kg).
            inertia (float): Moment of inertia about the vertical axis (kg*m^2).
            damping (list): List of damping coefficients [D_x, D_y, D_psi].
        """
        self.state = np.array(initial_state, dtype=np.float64)  # Initial state of the vessel
        self.m = mass  # Mass of the vessel
        self.Iz = inertia  # Moment of inertia about vertical axis
        self.D = damping  # Damping coefficients
        self.L = 1  # Distance from center of mass to thruster (m)

    def dynamics(self, control: np.ndarray):
        """
        Computes the dynamics of the vessel based on the current control inputs.

        Args:
            control (np.ndarray): Control input from the controller [u_1, u_2] - left and right thrusters.

        Returns:
            np.ndarray: Derivative of the vessel's state [dx, dy, dpsi, dVx, dVy, domega].
        """
        x, y, psi, Vx, Vy, omega = self.state
        dx = Vx * np.cos(psi) - Vy * np.sin(psi)
        dy = Vx * np.sin(psi) + Vy * np.cos(psi)
        dpsi = omega
        # Forces assumed directly proportional to control inputs
        dVx = (control[0] + control[1]) / self.m - self.D[0]*Vx
        dVy = 0 - self.D[1]*Vy  # Assuming negligible lateral thrust
        domega = self.L * (control[1] - control[0]) / self.Iz - self.D[2]*omega
        return np.array([dx, dy, dpsi, dVx, dVy, domega])

    def update_state(self, control, dt):
        """
        Updates the vessel's state based on the current control inputs and time step.

        Args:
            control (np.ndarray): Control input from the controller [u_1, u_2] - left and right thrusters.
            dt (float): Time step for the simulation.

        Updates the state in place.
        """
        self.state += dt * self.dynamics(control)

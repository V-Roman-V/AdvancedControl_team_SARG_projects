import numpy as np
from dataclasses import dataclass


@dataclass
class BoatState:
    """State of the boat."""
    x: float
    y: float
    psi: float
    Vx: float
    Vy: float
    omega: float

    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'BoatState':
        """Creates a BoatState object from a numpy array."""
        if len(arr) != 6:
            raise ValueError("Array must have exactly 6 elements.")
        return cls(arr[0], arr[1], arr[2], arr[3], arr[4], arr[5])

    def to_array(self) -> np.ndarray:
        """Converts the boat state to a numpy array."""
        return np.array([self.x, self.y, self.psi, self.Vx, self.Vy, self.omega])

    def update(self, derivatives: np.ndarray, dt: float) -> None:
        """Updates the boat state using Euler integration."""
        if len(derivatives) != 6:
            raise ValueError("Derivatives must have exactly 6 elements.")
        self.x += derivatives[0] * dt
        self.y += derivatives[1] * dt
        self.psi += derivatives[2] * dt
        self.Vx += derivatives[3] * dt
        self.Vy += derivatives[4] * dt
        self.omega += derivatives[5] * dt

@dataclass
class BoatParameters:
    """Parameters of the boat."""
    mass: float
    inertia: float
    damping: list
    L: float  # Distance from CoM to thruster

class Boat:
    """Base class for boat dynamics with thrusters."""
    def __init__(self, init_state: BoatState, params: BoatParameters):
        """
        Initializes the Boat object with initial conditions and system parameters.
        """
        self.state = init_state
        self.params = params

    def dynamics(self, control: np.ndarray) -> np.ndarray:
        """
        Computes the dynamics of the vessel based on the current control inputs.

        Returns:
            np.ndarray: Derivative of the vessel's state [dx, dy, dpsi, dVx, dVy, domega].
        """
        raise NotImplementedError("Dynamics method not implemented.")

    def _kinematics(self, Fx: float, Fy: float, M: float) -> np.ndarray:
        """
        Computes the kinematics of the vessel based on forces and moments.

        Returns:
            np.ndarray: Derivative of the vessel's state [dx, dy, dpsi, dVx, dVy, domega].
        """
        # Kinematics
        dx = self.state.Vx * np.cos(self.state.psi) - self.state.Vy * np.sin(self.state.psi)
        dy = self.state.Vx * np.sin(self.state.psi) + self.state.Vy * np.cos(self.state.psi)
        dpsi = self.state.omega
        dVx = (Fx / self.params.mass) - self.params.damping[0] * self.state.Vx
        dVy = (Fy / self.params.mass) - self.params.damping[1] * self.state.Vy
        domega = (M / self.params.inertia) - self.params.damping[2] * self.state.omega
        return np.array([dx, dy, dpsi, dVx, dVy, domega])

    def update_state(self, control: np.ndarray, dt: float) -> None:
        """Updates state using Euler integration."""
        derivatives = self.dynamics(control)
        self.state.update(derivatives, dt)


class DifferentialThrustBoat(Boat):
    """Boat with two fixed thrusters (left/right)."""
    def dynamics(self, control: np.ndarray) -> np.ndarray:
        Fx = control[0] + control[1]  # Sum of thrusts
        Fy = 0  # No lateral force
        M = self.params.L * (control[1] - control[0])  # Differential torque
        return self._kinematics(Fx, Fy, M)


class SteerableThrustBoat(Boat):
    """Boat with a single steerable thruster."""
    def dynamics(self, control: np.ndarray) -> np.ndarray:
        thrust, theta = control[0], control[1]
        Fx = thrust * np.cos(theta)
        Fy = thrust * np.sin(theta)
        M = thrust * self.params.L * np.sin(theta)  # Torque from offset
        return self._kinematics(Fx, Fy, M)
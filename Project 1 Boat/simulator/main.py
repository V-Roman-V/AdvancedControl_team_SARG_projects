import numpy as np
from boat import Boat
from control import Controller
from visualization import BoatVisualizer
from tqdm import tqdm


class Simulation():
    def __init__(self, fin_time, dt, mode = 'gif'):
        self.mode = mode
        self.dt = dt
        self.time = np.arange(0, fin_time, dt)

        # Trajectory
        self.trajectory = []
        self.desired_trajectory = []
        
        # visualizer
        self.gif_path = "simulation.gif"
        self.frame_numbers = 100
        self.update_vis_every_n_frame = len(self.time) // self.frame_numbers 
        self.visualizer: BoatVisualizer = None

        # Boat
        self.mass = 500
        self.inertia = 200
        self.damping = [0.5, 0.5, 0.1]
        self.boat: Boat = None

        # Control
        self.controller = Controller(
            mass=self.mass,
            inertia=self.inertia,
            damping=self.damping,
            control_limit=20.0,  # Increased for better maneuverability
            k_p_pos=0.8,         # More aggressive position tracking
            k_p_v=3.0,
            Vx_max=2.5,
            k_p_psi=0.8,        # Faster heading correction
            k_p_omega=3.0,
            omega_max=1.2
        )
        self.control_history = []

    def initialize(self, init_state, desired_trajectory):
        self.desired_trajectory = desired_trajectory
        self.visualizer =  BoatVisualizer(self.mode, desired_trajectory=desired_trajectory)
        self.boat = Boat(initial_state=init_state, mass=self.mass, inertia=self.inertia, damping=self.damping)

    def simulate(self):
        """
        The simulation computes control inputs at each time step and updates the vessel's state.
        Finally, it visualizes the actual trajectory and compares it with the desired trajectory.
        """
        desired_state = self.desired_trajectory[0]
        for i, t in enumerate(tqdm(self.time)):
            control_input = self.controller.compute_control(self.boat.state, desired_state)
            self.boat.update_state(control_input, self.dt)
            self.trajectory.append(self.boat.state.copy())
            self.control_history.append(control_input.copy())
            if i % self.update_vis_every_n_frame == 0:
                self.visualizer.update(self.trajectory, current_step=t, controls=self.control_history[-1])
        self.visualizer.finalize(self.trajectory, save_path=self.gif_path)

def main():
    """
    Entry point for the simulation. Initializes the vessel, controller, and runs the simulation loop.
    """
    T = 50
    dt = 0.01
    sim = Simulation(T, dt, 'gif')  # 'gif', 'realtime', 'final'

    # Initial and desired states: [x, y, psi, Vx, Vy, omega]
    init_state = [0, 0, 0, 0, 0, 0]
    desired_state = np.array([4, 4, 1, 0, 0, 0])
    sim.initialize(init_state, [desired_state])
    
    # start
    sim.simulate()

if __name__ == "__main__":
    main()

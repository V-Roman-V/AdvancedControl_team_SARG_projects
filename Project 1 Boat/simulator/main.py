import numpy as np
from boat import BoatState, BoatParameters, DifferentialThrustBoat, SteerableThrustBoat, Boat
from controllers import Controller, DifferentialController
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
        self.boat_parameters = BoatParameters(
            mass=500,
            inertia=200,
            damping=[0.5, 0.5, 0.1],
            L=1,
        )
        self.boat: Boat = None
        self.controller: Controller = None
        self.control_history = []

    def initialize(self, init_state: BoatState, desired_state: BoatState, control_limit=20):
        self.desired_state = desired_state.to_array()
        self.visualizer = BoatVisualizer(self.mode, desired_trajectory=[self.desired_state], control_limit=control_limit)
        self.boat = DifferentialThrustBoat(init_state, self.boat_parameters)
        self.controller = DifferentialController(self.boat_parameters, control_limit=control_limit)

    def simulate(self):
        """
        The simulation computes control inputs at each time step and updates the vessel's state.
        Finally, it visualizes the actual trajectory and compares it with the desired trajectory.
        """
        for i, t in enumerate(tqdm(self.time)):
            boat_state = self.boat.state.to_array()
            control_input = self.controller.compute_control(boat_state, self.desired_state)
            self.boat.update_state(control_input, self.dt)
            self.trajectory.append(boat_state.copy())
            self.control_history.append(control_input.copy())
            if i % self.update_vis_every_n_frame == 0:
                self.visualizer.update(self.trajectory, current_step=t, controls=self.control_history[-1])
        self.visualizer.finalize(self.trajectory, save_path=self.gif_path)

def main():
    """
    Entry point for the simulation. Initializes the vessel, controller, and runs the simulation loop.
    """
    T = 200
    dt = 0.01
    sim = Simulation(T, dt, 'realtime')  # 'gif', 'realtime', 'final'

    # Initial and desired states: [x, y, psi, Vx, Vy, omega]
    init_state = BoatState(x=0, y=0, psi=0, Vx=0, Vy=0, omega=0)
    desired_state = BoatState(x=10, y=10, psi=0, Vx=0, Vy=0, omega=0)
    sim.initialize(init_state, desired_state, control_limit=20)
    
    # start
    sim.simulate()

if __name__ == "__main__":
    main()

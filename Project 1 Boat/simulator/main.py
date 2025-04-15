import numpy as np
from boat import BoatState, BoatParameters, DifferentialThrustBoat, SteerableThrustBoat, Boat
from controllers import Controller, DifferentialController, SteeringController
from visualization import BoatVisualizer
from tqdm import tqdm
from typing import List

class Simulation():
    def __init__(self, fin_time, dt, mode='gif', boat_types: List[str] = ['differential']):
        self.mode = mode
        self.dt = dt
        self.time = np.arange(0, fin_time, dt)
        self.boat_types = boat_types
        self.num_boats = len(boat_types)

        # Trajectories and controls for all boats
        self.trajectories = [[] for _ in range(self.num_boats)]
        self.desired_trajectories = []
        self.control_histories = [[] for _ in range(self.num_boats)]

        # Visualization
        self.gif_path = "simulation.gif"
        self.frame_numbers = 100
        self.update_vis_every_n_frame = len(self.time) // self.frame_numbers 
        self.visualizer: BoatVisualizer = None

        # Boat parameters (same for all boats for simplicity)
        self.boat_parameters = BoatParameters(
            mass=500,
            inertia=200,
            damping=[0.5, 0.5, 0.1],
            L=1,
        )
        self.boats: List[Boat] = []
        self.controllers: List[Controller] = []

        # Control limits
        self.dif_control_limit = 20
        self.steer_control_limit = [40, np.pi]

    def initialize(self, init_states: List[BoatState], desired_states: List[BoatState]):
        # Ensure desired_trajectories is a list of 2D arrays (x, y)
        self.desired_trajectories = [np.array([ds.x, ds.y]) for ds in desired_states]
        
        control_limits = []
        for i, boat_type in enumerate(self.boat_types):
            if boat_type == 'differential':
                boat = DifferentialThrustBoat(init_states[i], self.boat_parameters)
                controller = DifferentialController(self.boat_parameters, control_limit=self.dif_control_limit)
                control_limits.append(self.dif_control_limit)
            elif boat_type == 'steerable':
                boat = SteerableThrustBoat(init_states[i], self.boat_parameters)
                controller = SteeringController(self.boat_parameters, control_limit=self.steer_control_limit)
                control_limits.append(self.steer_control_limit[0])
            
            self.boats.append(boat)
            self.controllers.append(controller)

        self.visualizer = BoatVisualizer(
            self.mode, 
            desired_trajectories=self.desired_trajectories,
            control_limits=control_limits,
            num_boats=self.num_boats
        )

    def simulate(self):
        for i, t in enumerate(tqdm(self.time)):
            current_controls = []
            for boat_idx in range(self.num_boats):
                # Get current state and compute control
                boat_state = self.boats[boat_idx].state.to_array()
                control_input = self.controllers[boat_idx].compute_control(
                    boat_state, 
                    self.desired_trajectories[boat_idx]
                )
                
                # Update boat state and store trajectory (ensure full state is stored)
                self.boats[boat_idx].update_state(control_input, self.dt)
                self.trajectories[boat_idx].append(boat_state[:3])  # Ensure x, y, psi are stored
                self.control_histories[boat_idx].append(control_input.copy())
                current_controls.append(control_input.copy())

            # Update visualization
            if i % self.update_vis_every_n_frame == 0:
                self.visualizer.update(
                    [t[-1] for t in self.trajectories],
                    current_step=t,
                    controls=[c[-1] for c in self.control_histories]
                )
        
        self.visualizer.finalize(
            self.trajectories,
            save_path=self.gif_path
        )


def main():
    T = 400
    dt = 0.1
    boat_types = ['differential', 'steerable','differential']
    
    sim = Simulation(T, dt, 'realtime', boat_types)

    # Initial and desired states for each boat
    init_states = [
        BoatState(x=0, y=0, psi=0, Vx=0, Vy=0, omega=0),
        BoatState(x=5, y=-5, psi=np.pi/2, Vx=0, Vy=0, omega=0),
        BoatState(x=15, y=15, psi=np.pi/2, Vx=0, Vy=0, omega=0)
    ]
    
    desired_states = [
        BoatState(x=10, y=10, psi=0, Vx=0, Vy=0, omega=0),
        BoatState(x=10, y=10, psi=np.pi, Vx=0, Vy=0, omega=0),
        BoatState(x=10, y=10, psi=np.pi, Vx=0, Vy=0, omega=0)

    ]
    
    sim.initialize(init_states, desired_states)
    sim.simulate()

if __name__ == "__main__":
    main()

import numpy as np
from boat import BoatState, BoatParameters, DifferentialThrustBoat, SteerableThrustBoat
from controllers import DifferentialController, SteeringController
from visualization import BoatVisualizer
from tqdm import tqdm

class Simulation:
    def __init__(self, fin_time, dt, mode='realtime', wind_velocity = (0, 0), boat_types=None):
        self.dt = dt
        self.time = np.arange(0, fin_time, dt)
        self.wind_velocity = np.array(wind_velocity)
        self.boat_types = boat_types
        self.num_boats = len(boat_types)
        self.trajectories = [[] for _ in range(self.num_boats)]
        self.control_histories = [[] for _ in range(self.num_boats)]
        self.boats = []
        self.controllers = []
        self.visualizer = None
        self.control_limits = {'differential': 20, 'steerable': [40, np.pi/2]}
        self.boat_parameters = BoatParameters(mass=500, inertia=200, damping=[0.5, 0.5, 0.1], L=1)
        self.mode=mode
        self.frame_numbers = 80
        self.update_vis_every_n_frame = len(self.time) // self.frame_numbers 

    def initialize(self, init_states, desired_states):
        desired_trajs = [[ds.x, ds.y] for ds in desired_states]
        for i, boat_type in enumerate(self.boat_types):
            state = init_states[i]
            if boat_type == 'differential':
                self.boats.append(DifferentialThrustBoat(state, self.boat_parameters, self.wind_velocity))
                self.controllers.append(DifferentialController(self.boat_parameters, self.control_limits['differential']))
            else:
                self.boats.append(SteerableThrustBoat(state, self.boat_parameters, self.wind_velocity))
                self.controllers.append(SteeringController(self.boat_parameters, self.control_limits['steerable']))

        self.visualizer = BoatVisualizer(
            mode=self.mode,
            desired_trajectories=desired_trajs,
            control_limits=self.control_limits,
            boat_types=self.boat_types,
            wind_velocity=self.wind_velocity,
        )

    def print_wind(self):
        for type, boat in zip(self.boat_types, self.boats):
            print(f"{boat.state.Wx:.3f}, {boat.state.Wy:.3f}, | {type}")
        print('-----')
        print(self.wind_velocity)

    def simulate(self):
        for frame, t in enumerate(tqdm(self.time)):
            states, controls = [], []
            for i, boat in enumerate(self.boats):
                state = boat.state.to_array()
                u, wind_derivatives = self.controllers[i].compute_control(state, self.visualizer.desired_trajs[i])
                boat.update_state(u, wind_derivatives, self.dt)
                self.trajectories[i].append(state)
                states.append(state[:3])
                controls.append(u)
                self.control_histories[i].append(u)
            # Update visualization
            if frame % self.update_vis_every_n_frame == 0:
                self.visualizer.update(states, self.trajectories, t, controls, self.dt * self.update_vis_every_n_frame)
        print("finalize")
        self.visualizer.create_target_phase_plot(self.trajectories, self.visualizer.desired_trajs, save_path='target_phase_plot.png')
        self.visualizer.create_estimated_wind_plot(self.trajectories, self.wind_velocity, save_path='wind_estimates_plot.png')
        self.visualizer.create_control_plot(self.control_histories, self.boat_types, save_path='control_plot.png')
        self.visualizer.finalize()

def generate_random_boats(num_boats, seed=42, goal=(0, 0)):
    """
    Generate random boat types, initial states, and desired states.
    :param num_boats: Number of boats to generate
    :param seed: Seed for reproducibility
    :param goal: Tuple (x, y) target position
    :return: boat_types, init_states, desired_states
    """
    np.random.seed(seed)
    boat_types = np.random.choice(['differential', 'steerable'], size=num_boats).tolist()
    # boat_types = ['steerable']
    # boat_types = ['differential']

    init_states = []
    desired_states = []

    for _ in range(num_boats):
        x = np.random.uniform(-10, 10)
        y = np.random.uniform(-10, 10)
        psi = np.random.uniform(-np.pi, np.pi)
        init_states.append(BoatState(x, y, psi, 0, 0, 0, 0, 0))

        # Slightly jitter the goal to make them non-identical
        dx = goal[0] # + np.random.uniform(-1.0, 1.0)
        dy = goal[1] # + np.random.uniform(-1.0, 1.0)
        desired_states.append(BoatState(dx, dy, np.pi, 0, 0, 0, 0, 0))

    return boat_types, init_states, desired_states

def main():
    T, dt = 300, 1
    wind_velocity = (0.04, -0.03)
    boat_types, init_states, desired_states = generate_random_boats(20)
    sim = Simulation(T, dt, 'final', wind_velocity, boat_types)
    sim.initialize(init_states, desired_states)
    sim.simulate()
    sim.print_wind()

if __name__ == "__main__":
    main()

import numpy as np
from boat import BoatState, BoatParameters, DifferentialThrustBoat, SteerableThrustBoat
from controllers import DifferentialController, SteeringController
from visualization import BoatVisualizer
from tqdm import tqdm

class Simulation:
    def __init__(self, fin_time, dt, mode='realtime', boat_types=None):
        self.dt = dt
        self.time = np.arange(0, fin_time, dt)
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

    def initialize(self, init_states, desired_states):
        desired_trajs = [[ds.x, ds.y] for ds in desired_states]
        for i, boat_type in enumerate(self.boat_types):
            state = init_states[i]
            if boat_type == 'differential':
                self.boats.append(DifferentialThrustBoat(state, self.boat_parameters))
                self.controllers.append(DifferentialController(self.boat_parameters, self.control_limits['differential']))
            else:
                self.boats.append(SteerableThrustBoat(state, self.boat_parameters))
                self.controllers.append(SteeringController(self.boat_parameters, self.control_limits['steerable']))

        self.visualizer = BoatVisualizer(
            mode=self.mode,
            desired_trajectories=desired_trajs,
            control_limits=self.control_limits,
            boat_types=self.boat_types
        )

    def simulate(self):
        for t in tqdm(self.time):
            states, controls = [], []
            for i, boat in enumerate(self.boats):
                state = boat.state.to_array()
                u = self.controllers[i].compute_control(state, self.visualizer.desired_trajs[i])
                boat.update_state(u, self.dt)
                self.trajectories[i].append(state)
                states.append(state[:3])
                controls.append(u)
            self.visualizer.update(states, self.trajectories, t, controls)
        print("finalize")
        self.visualizer.create_target_phase_plot(self.trajectories, self.visualizer.desired_trajs, save_path='target_phase_plot.png')
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

    init_states = []
    desired_states = []

    for _ in range(num_boats):
        x = np.random.uniform(-10, 10)
        y = np.random.uniform(-10, 10)
        psi = np.random.uniform(-np.pi, np.pi)
        init_states.append(BoatState(x, y, psi, 0, 0, 0))

        # Slightly jitter the goal to make them non-identical
        dx = goal[0] # + np.random.uniform(-1.0, 1.0)
        dy = goal[1] # + np.random.uniform(-1.0, 1.0)
        desired_states.append(BoatState(dx, dy, np.pi, 0, 0, 0))

    return boat_types, init_states, desired_states

def main():
    T, dt = 500, 1
    boat_types, init_states, desired_states = generate_random_boats(60)
    sim = Simulation(T, dt, 'final', boat_types)
    sim.initialize(init_states, desired_states)
    sim.simulate()

if __name__ == "__main__":
    main()

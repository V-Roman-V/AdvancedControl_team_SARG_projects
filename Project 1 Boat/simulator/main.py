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

def main():
    T, dt = 200, 1
    boat_types = ['differential', 'steerable', 'steerable', 'differential', 'differential']
    sim = Simulation(T, dt, 'gif', boat_types)

    init_states = [
        BoatState(0, 0, 0, 0, 0, 0),
        BoatState(5, -5, np.pi/2, 0, 0, 0),
        BoatState(15, 15, np.pi/2, 0, 0, 0),
        BoatState(-15, 15, np.pi/2, 0, 0, 0),
        BoatState(15, 0, np.pi/2, 0, 0, 0)
    ]

    desired_states = [
        BoatState(10, 10, 0, 0, 0, 0),
        BoatState(10, 10, np.pi, 0, 0, 0),
        BoatState(10, 10, np.pi, 0, 0, 0),
        BoatState(10, 10, np.pi, 0, 0, 0),
        BoatState(10, 10, np.pi, 0, 0, 0)
    ]

    sim.initialize(init_states, desired_states)
    sim.simulate()

if __name__ == "__main__":
    main()

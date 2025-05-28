import numpy as np
from cartpole import CartPole
from controller import CartPoleController
from visualizer import CartPoleVisualizer
from tqdm import tqdm

class Simulation:
    def __init__(self, T=10.0, dt=0.02, mode='realtime'):
        self.T = T
        self.dt = dt
        self.mode = mode
        self.time = np.arange(0, T, dt)
        self.cartpole = None
        self.controller = None
        self.visualizer = CartPoleVisualizer(mode=mode)
        self.trajectory = []
        self.controls = []
        self.frame_numbers = 150
        self.update_vis_every_n_frame = len(self.time) // self.frame_numbers

    def initialize(self, init_state):
        self.cartpole = CartPole(init_state)
        self.controller = CartPoleController()

    def simulate(self):
        for frame_idx, t in enumerate(tqdm(self.time)):
            state = self.cartpole.state
            control = self.controller.compute_control(state)
            self.cartpole.update(control, self.dt)

            self.trajectory.append(state.copy())
            self.controls.append(control)

            if frame_idx % self.update_vis_every_n_frame == 0:
                self.visualizer.update(state, t)

        self.visualizer.finalize('./cartpole.gif')


def main():
    # Initial state: [x, x_dot, theta, theta_dot]
    init_state = np.array([0.0, 0.0, np.pi - 0.05, 0.0])  # Slightly perturbed downward
    sim = Simulation(T=10.0, dt=0.02, mode='gif')
    sim.initialize(init_state)
    sim.simulate()

if __name__ == "__main__":
    main()

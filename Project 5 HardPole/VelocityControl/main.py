import numpy as np
from cartpole import CartPole
from controller import Controller, ControlParams
from visualizer import CartPoleVisualizer
from tqdm import tqdm

class Simulation:
    def __init__(self, T=10.0, dt=0.02, frame_numbers = 150, mode='realtime', label_text=""):
        self.T = T
        self.dt = dt
        self.mode = mode
        self.time = np.arange(0, T, dt)
        self.controller = None
        self.cartpole = None
        self.visualizer = CartPoleVisualizer(mode=mode, label_text=label_text)
        self.trajectory = []
        self.controls = []
        self.frame_numbers = frame_numbers
        self.update_vis_every_n_frame = len(self.time) // self.frame_numbers

    def initialize(self,cartpole:CartPole, controller:Controller):
        self.controller = controller
        self.cartpole = cartpole

    def simulate(self, wait_time=0.2):
        for frame_idx, t in enumerate(tqdm(self.time)):
            state = self.cartpole.state
            
            control = 0
            if t > wait_time:
                control = self.controller.compute_control(state, self.dt)
                
            self.cartpole.update(control, self.dt)

            self.trajectory.append(state.copy())
            self.controls.append(control)

            if frame_idx % self.update_vis_every_n_frame == 0:
                self.visualizer.update(state, control, t)

        self.visualizer.finalize()

def main():
    #create cartpole instance
    init_state = np.array([-0.1, 0.0, np.pi+0.15, 0.0])  # Start nearly hanging downward
    # init_state = np.array([-0.1, 0.0, 0.15, 0.0])  # Start nearly hanging upward
    cartpole = CartPole(init_state)

    #create controller instance
    mpc_monte = ControlParams.MPCMonteParams(horizon_seconds=0.45, samples=800, x_limit=0.15, force_step_part=0.1, weight_theta=2000, weight_x=100.0, weight_theta_dot=0.1, weight_x_dot=0.2)
    params = ControlParams(mpc_monte=mpc_monte)

    method = "mpc_montecarlo"  # "pd", "energy", "hybrid"
    controller = Controller(method=method, params=params)

    sim = Simulation(T=9, dt=0.007, frame_numbers=200, label_text="MPC MONTECARLO\n(like real system)", mode='realtime')
    sim.initialize(cartpole, controller)
    sim.simulate(wait_time = 0.0)

if __name__ == "__main__":
    main()
   
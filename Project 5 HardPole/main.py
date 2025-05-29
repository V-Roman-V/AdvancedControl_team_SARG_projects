import numpy as np
from cartpole import CartPole, CartPoleParams
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
    init_state = np.array([0.1, 0.0, np.pi+0.05, 0.0])  # Start nearly hanging downward
    # init_state = np.array([-1, 0.0, 0.15, 0.0])  # Start nearly hanging upward
    cartpole = CartPole(init_state)

    #create controller instance
    pd = ControlParams.PDParams(k_theta_p=300.0, k_theta_d=100.0, k_x_p=1.0, k_x_d=-50.0, k_x_i=100, k_x_i_dur=0.2, switch_angle_deg=45.0) # fast stable
    energy = ControlParams.EnergyParams(k_energy=15.0)
    hybrid = ControlParams.HybridParams(switch_angle_deg=45.0)
    mpc_monte = ControlParams.MPCMonteParams(horizon_seconds=0.45, samples=550, x_limit=0.5, force_step_part=0.03, weight_theta=15000, weight_x=15.0, weight_theta_dot=1.5, weight_x_dot=0.0)
    params = ControlParams(pd=pd, energy=energy,hybrid=hybrid, mpc_monte=mpc_monte)

    method = "mpc_montecarlo"  # "pd", "energy", "hybrid"
    controller = Controller(method=method, params=params)

    sim = Simulation(T=10, dt=0.007, frame_numbers=200, label_text=method, mode='gif')
    sim.initialize(cartpole, controller)
    sim.simulate(wait_time = 0.0)

if __name__ == "__main__":
    main()
   
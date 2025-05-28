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
    init_state = np.array([-1, 0.0, np.pi+0.05, 0.0])  # Start nearly hanging downward
    cartpole = CartPole(init_state)

    #create controller instance
    # pd = ControlParams.PDParams(k_theta_p=35000.0, k_theta_d=8000.0, k_theta_i=1, k_theta_i_dur=1.5, k_x_p=150.0, k_x_d=400.0)
    # pd = ControlParams.PDParams(k_theta_p=1500.0, k_theta_d=100.0, k_theta_i=100.6, k_theta_i_dur=0.2, k_x_p=0.0, k_x_d=0.0)
    # pd = ControlParams.PDParams(k_theta_p=0.0, k_theta_d=0.0, k_theta_i=0, k_theta_i_dur=0.0, k_x_p=2000.0, k_x_d=450.0)
    # pd = ControlParams.PDParams(k_theta_p=1500.0, k_theta_d=100.0, k_theta_i=100, k_theta_i_dur=0.2, k_x_p=1.0, k_x_d=-50.0) # stable
    pd = ControlParams.PDParams(k_theta_p=300.0, k_theta_d=100.0, k_x_p=1.0, k_x_d=-50.0, k_x_i=100, k_x_i_dur=0.2) # fast stable
    energy = ControlParams.EnergyParams(k_energy=15.0)
    hybrid = ControlParams.HybridParams(switch_angle_deg=45.0)
    params = ControlParams(pd=pd, energy=energy,hybrid=hybrid)

    method = "pd"  # "pd", "energy", "hybrid"
    controller = Controller(method=method, params=params)

    sim = Simulation(T=6, dt=0.001, frame_numbers=120, label_text="PID", mode='gif')
    sim.initialize(cartpole, controller)
    sim.simulate(wait_time = 1.0)

if __name__ == "__main__":
    main()
   
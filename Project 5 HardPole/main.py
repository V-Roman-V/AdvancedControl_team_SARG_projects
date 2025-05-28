import numpy as np
from cartpole import CartPole, CartPoleParams
from controller import Controller, ControlParams
from visualizer import CartPoleVisualizer
from tqdm import tqdm

class Simulation:
    def __init__(self, T=10.0, dt=0.02, frame_numbers = 150, mode='realtime'):
        self.T = T
        self.dt = dt
        self.mode = mode
        self.time = np.arange(0, T, dt)
        self.controller = None
        self.cartpole = None
        self.visualizer = CartPoleVisualizer(mode=mode)
        self.trajectory = []
        self.controls = []
        self.frame_numbers = frame_numbers
        self.update_vis_every_n_frame = len(self.time) // self.frame_numbers

    def initialize(self,cartpole:CartPole, controller:Controller):
        self.controller = controller
        self.cartpole = cartpole

    def simulate(self):
        for frame_idx, t in enumerate(tqdm(self.time)):
            state = self.cartpole.state
            control = self.controller.compute_control(state, self.dt)
            self.cartpole.update(control, self.dt)

            self.trajectory.append(state.copy())
            self.controls.append(control)

            if frame_idx % self.update_vis_every_n_frame == 0:
                self.visualizer.update(state, t)

        self.visualizer.finalize('./cartpole.gif')

def main():
    #create cartpole instance
    init_state = np.array([-1, 0.0, np.pi/2+0.5, 0.0])  # Start nearly hanging downward
    cartpole = CartPole(init_state)

    #create controller instance
    pd = ControlParams.PDParams(k_theta_p=35000.0, k_theta_d=8000.0, k_theta_i=1, k_theta_i_dur=1.5, k_x_p=150.0, k_x_d=400.0)
    energy = ControlParams.EnergyParams(k_energy=25.0)
    hybrid = ControlParams.HybridParams(switch_angle_deg=45.0)
    params = ControlParams(pd=pd, energy=energy,hybrid=hybrid)

    controller = Controller(method="pd", params=params)

    sim = Simulation(T=60.0, dt=0.001, frame_numbers=400, mode='gif')
    sim.initialize(cartpole, controller)
    sim.simulate()

if __name__ == "__main__":
    main()

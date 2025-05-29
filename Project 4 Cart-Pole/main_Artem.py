import time
import numpy as np

from UART import UART
from cart_pole import State, CartPole as MyCartPole
from converter import control_to_speed, speed_to_control
from MPC_controller import NMPCControllerSC, SwingNMPCControllerSC


import numpy as np
from dataclasses import dataclass
@dataclass
class CartPoleParams:
    M: float = 4.5        # cart mass (kg)
    m: float = 0.300      # pole mass (kg)
    l: float = 0.227      # pole length (m)
    b_c: float = 1.0      # viscous friction (cart)
    f_c: float = 1.4      # Coulomb friction (cart)
    b_p: float = 0.0028   # viscous friction (pole)
    f_p: float = 0.0095   # Coulomb friction (pole)
    K_p: float = 100      # Virtual Force
    max_speed: float = 1.2  # max speed of the cart (m/s)
    g: float = 9.81  # max speed of the cart (m/s)

class CartPole:
    g = 9.81

    def __init__(self, init_state: np.ndarray, params: CartPoleParams = None):
        self.state = init_state.astype(np.float64)

        if params is None:
            params = CartPoleParams()
        self.params_obj = params
        self.params = np.array([
            params.M,       # M: cart mass (kg)
            params.m,       # m: pole mass (kg)
            params.l,       # l: pole length (m)
            params.b_c,     # b_c: viscous friction (cart)
            params.f_c,     # f_c: Coulomb friction (cart)
            params.b_p,     # b_p: viscous friction (pole)
            params.f_p,     # f_p: Coulomb friction (pole)
            params.K_p      # K_p: Virtual Force  
        ])

    def get_dynamic(self, params, state, u_speed=0.0):
        _, x_dot, theta, theta_dot = state
        g = self.g
        M, m, L, b_c, f_c, b_p, f_p, K_pf = params


        # Intermediate terms
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        sgn_x_dot = np.sign(x_dot)
        sgn_theta_dot = np.sign(theta_dot)

        # Friction forces
        F_fric = b_c * x_dot + f_c * sgn_x_dot
        T_fric = b_p * theta_dot + f_p * sgn_theta_dot

        # Virtual Force
        u_f = K_pf * (u_speed - x_dot)

        D = M + m
        mlcos = m * L * cos_theta

        # Compute effective inertia term for theta equation
        alpha = m * L**2 - (mlcos**2) / D

        # Numerator of angular acceleration
        beta = (
            -m * g * L * sin_theta
            - (mlcos / D) * (-M*u_f + F_fric + m * L * theta_dot**2 * sin_theta)
            - T_fric
        )

        # Angular acceleration
        theta_ddot = beta / alpha

        # Linear acceleration
        pole_force = mlcos * theta_ddot - m * L * theta_dot**2 * sin_theta
        x_ddot = u_f + pole_force / 100

        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])
    
    @staticmethod
    def dynamics_batch_orig(params: CartPoleParams, states: np.ndarray, u_speeds: np.ndarray) -> np.ndarray:
        """
        Vectorized dynamics for a batch of states and u_speeds.
        states: shape (N, 4) -> [x, x_dot, theta, theta_dot]
        u_speeds: shape (N,)
        Returns: shape (N, 4) -> [x_dot, x_ddot, theta_dot, theta_ddot]
        """
        x_dot = states[:, 1]
        theta = states[:, 2]
        theta_dot = states[:, 3]

        g = 9.81
        np_params = np.array([
            params.M,       # M: cart mass (kg)
            params.m,       # m: pole mass (kg)
            params.l,       # l: pole length (m)
            params.b_c,     # b_c: viscous friction (cart)
            params.f_c,     # f_c: Coulomb friction (cart)
            params.b_p,     # b_p: viscous friction (pole)
            params.f_p,     # f_p: Coulomb friction (pole)
            params.K_p      # K_p: Virtual Force  
        ])
        M, m, L, b_c, f_c, b_p, f_p, K_pf = np_params

        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        sgn_x_dot = np.sign(x_dot)
        sgn_theta_dot = np.sign(theta_dot)

        F_fric = b_c * x_dot + f_c * sgn_x_dot
        T_fric = b_p * theta_dot + f_p * sgn_theta_dot
        u_f = K_pf * (u_speeds - x_dot)

        D = M + m
        mlcos = m * L * cos_theta
        alpha = m * L**2 - (mlcos**2) / D

        beta = (
            -m * g * L * sin_theta
            - (mlcos / D) * (-M * u_f + F_fric + m * L * theta_dot**2 * sin_theta)
            - T_fric
        )

        theta_ddot = beta / alpha
        pole_force = mlcos * theta_ddot - m * L * theta_dot**2 * sin_theta
        x_ddot = u_f + pole_force / 100

        derivs = np.stack([x_dot, x_ddot, theta_dot, theta_ddot], axis=-1)
        return derivs
    
    @staticmethod
    def dynamics_batch(params: CartPoleParams, states: np.ndarray, u_speeds: np.ndarray) -> np.ndarray:
        roman_state = states.copy()
        roman_state[:, 0] = -roman_state[:, 0]
        roman_state[:, 1] = -roman_state[:, 1]
        roman_state[:, 2] = np.pi - roman_state[:, 2]
        roman_state[:, 3] = -roman_state[:, 3]
        roman_u_speeds = -u_speeds
        roman_derivs = CartPole.dynamics_batch_orig(params, roman_state, roman_u_speeds)
        artem_derivs = roman_derivs.copy()
        artem_derivs[:, 0] = -artem_derivs[:, 0]
        artem_derivs[:, 1] = -artem_derivs[:, 1]
        artem_derivs[:, 2] = -artem_derivs[:, 2]
        artem_derivs[:, 3] = -artem_derivs[:, 3]
        return artem_derivs
        

    def update(self, speed: float, dt: float):
        speed = np.clip(speed, -self.params_obj.max_speed, self.params_obj.max_speed)

        roman_state = self.state.copy()
        roman_state[0] = - roman_state[0]
        roman_state[1] = - roman_state[1]
        roman_state[2] = np.pi - roman_state[2]
        roman_state[3] = - roman_state[3]

        roman_speed = - speed

        roman_derivs = self.get_dynamic(self.params, roman_state, roman_speed)
        artem_derivs = roman_derivs.copy()
        artem_derivs[0] = - artem_derivs[0]
        artem_derivs[1] = - artem_derivs[1]
        artem_derivs[2] = - artem_derivs[2]
        artem_derivs[3] = - artem_derivs[3]

        self.state += artem_derivs * dt
        self.state[2] = (self.state[2] + np.pi) % (2 * np.pi) - np.pi  # wrap theta ∈ [-π, π]

@dataclass
class ControlParams:
    @dataclass
    class MPCMonteParams:
        horizon_seconds: int = 0.5
        samples: int = 10000
        x_limit: float = 1 # limit for x position
        force_step_part: float = 0.1 # part of max force

        weight_theta: float = 10.0
        weight_theta_dot: float = 0.1
        weight_x: float = 1.0
        weight_x_dot: float = 0.1

    mpc_monte: MPCMonteParams = None

class Controller:
    def __init__(self, method: str = "mpc_montecarlo", params: ControlParams = None, cartpole_params: CartPoleParams = None):
        super().__init__()
        self.cartpole_params = cartpole_params if cartpole_params else CartPoleParams()
        self.params = params if params else ControlParams()
        self.method = method.lower()

    def _wrap_angle(self, angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _wrap_angle_batch(self, angles: np.ndarray) -> np.ndarray:
        return (angles + np.pi) % (2 * np.pi) - np.pi

    def compute_control(self, state: np.ndarray, dt: float = 0.02) -> float:
        control = 0
        if self.method == "mpc_montecarlo":
            control = self._compute_mpc_montecarlo_control(state, dt)
        else:
            raise ValueError(f"Unknown control method: {self.method}")
        return np.clip(control, -self.cartpole_params.max_speed, self.cartpole_params.max_speed)

    def _compute_mpc_montecarlo_control(self, state: np.ndarray,
                                            dt: float = 0.02) -> float:
        p    = self.params.mpc_monte
        N    = int(p.samples)
        H    = int(p.horizon_seconds / dt)       # steps in horizon
        m_f  = self.cartpole_params.max_speed
        fspd = m_f * p.force_step_part          # Δu range

        rng  = np.random.default_rng()

        # allocate once
        states = np.repeat(state[None, :], N, axis=0)   # (N,4)
        u0     = rng.normal(0, m_f / 2, N)              # initial force, normal distribution
        u      = u0.copy()

        # all random Δu for the entire horizon except the first step
        du_all = rng.normal(0, fspd / 2, (N, H-1))

        for k in range(H):
            # apply control
            vel   = CartPole.dynamics_batch(self.cartpole_params, states, u)
            states += dt * vel
            states[:, 2] = (states[:, 2] + np.pi) % (2*np.pi) - np.pi  # wrap θ
            states[np.abs(states[:, 0]) > p.x_limit][:,0] = np.inf  # ruine the pose

            if k < H-1:       # update force for next step
                u += du_all[:, k]
                np.clip(u, -m_f, m_f, out=u)

        # cost
        x, x_dot, th, th_dot = states.T
        th = (th + np.pi) % (2*np.pi) - np.pi
        cost = (p.weight_theta*th**2 + p.weight_theta_dot*th_dot**2 +
                p.weight_x*x**2     + p.weight_x_dot*x_dot**2)
        cost[np.abs(x) > p.x_limit] = np.inf

        best = cost.argmin()
        return float(u0[best])            # u0 is the action to apply


# def mainArtem():
#     mpc_monte = ControlParams.MPCMonteParams(horizon_seconds=0.45, samples=800, x_limit=0.15, force_step_part=0.1, weight_theta=2000, weight_x=100.0, weight_theta_dot=0.1, weight_x_dot=0.2)
#     params = ControlParams(mpc_monte=mpc_monte)
#     controller = Controller(method="mpc_montecarlo", params=params)

#     state = [0.0, 0.0, np.pi + 0.15, 0.0] 
#     dt = 0.07
#     control  = controller.compute_control(state=state, dt=dt)

   

def main():
    uart = UART()

    # mpc_monte = ControlParams.MPCMonteParams(horizon_seconds=0.134, samples=3500, x_limit=0.18, force_step_part=0.02, weight_theta=25000, weight_x=100.0, weight_theta_dot=5.1, weight_x_dot=0.2)
    # mpc_monte = ControlParams.MPCMonteParams(horizon_seconds=0.135, samples=3500, x_limit=0.18, force_step_part=0.015, weight_theta=25000, weight_x=100.0, weight_theta_dot=5.5, weight_x_dot=0.0)
    mpc_monte = ControlParams.MPCMonteParams(horizon_seconds=0.1, samples=3500, x_limit=0.18, force_step_part=0.015, weight_theta=25000, weight_x=100.0, weight_theta_dot=800.5, weight_x_dot=0.0)
    # mpc_monte = ControlParams.MPCMonteParams(horizon_seconds=0.025, samples=8500, x_limit=0.18, force_step_part=0.008, weight_theta=25000, weight_x=100.0, weight_theta_dot=80.5, weight_x_dot=0.0)
    # mpc_monte = ControlParams.MPCMonteParams(horizon_seconds=0.025, samples=8500, x_limit=0.18, force_step_part=0.008, weight_theta=25000, weight_x=100.0, weight_theta_dot=80.5, weight_x_dot=0.0)
    params = ControlParams(mpc_monte=mpc_monte)
    controller = Controller(method="mpc_montecarlo", params=params)

    for i in range(50):
        state = uart.wait_until_state()
        print("     ",state)

    # Parameters
    Thetha_shift = 0.0     # shift thetha value
    print("Start work!")
    uart.send_cart_velocity(0.8, verbose=True)
    time.sleep(0.10)
    uart.send_cart_velocity(-0.8, verbose=True)
    time.sleep(0.10)
    try:
        while True:
            # read state
            state = uart.wait_until_state()

            # Correcting angle value
            state.theta += Thetha_shift
            print("     ",state)

            # # Safety
            if abs(state.x) > 0.25:
                uart.send_cart_velocity(0)
                print(f"Reach border", state)
                time.sleep(1)
                break 

            # MPC control
            start = time.time()

            short_state = np.array([state.x, state.x_dot, state.theta, state.theta_dot])
            u_opt  = controller.compute_control(state=short_state, dt=state.dt)
            
            print(f"Optimal Control:", u_opt, f" dt = {(time.time() - start):.3f}")

            # send velocity
            uart.send_cart_velocity(u_opt, verbose=True)
    except KeyboardInterrupt:
        print("\nCtrl-C detected. Sending zero speed and exiting...")
        uart.send_cart_velocity(0)
        time.sleep(1)

if __name__ == "__main__":
    main()

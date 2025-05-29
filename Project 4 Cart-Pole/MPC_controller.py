import numpy as np
from scipy.optimize import minimize
from cart_pole import CartPole, State

def short_state_to_State(X, dt) -> State:
    return State(
        timestamp=0,
        dt = dt,
        x=X[0],
        x_dot=X[1],
        theta=X[2],
        theta_dot=X[3],
        old_ctrl=0,
    )

class NMPCControllerSC:
    def __init__(self, cartpole: CartPole, horizon=10, dt=0.007, num_iterations=100, u_lim=(-0.6, 0.6)):
        self.cartpole = cartpole
        self.horizon  = horizon
        self.dt       = dt
        self.u_lim    = u_lim
        self.num_iterations = num_iterations
        self.costs   = [5, 12, 0.7]  # [position, angle, effort]

        # a single State instance we’ll overwrite each step
        self._state = State(timestamp=0, dt=dt, x=0, x_dot=0, theta=0, theta_dot=0, old_ctrl=0)

    def _total_cost(self, u_seq, init_state: State):
        s = np.array([init_state.x, init_state.x_dot, init_state.theta, init_state.theta_dot])
        total = 0.0
        c0, c1, c2 = self.costs

        for u in u_seq:
            # update our scratch-State and compute derivatives
            self._state.x,      self._state.x_dot      = s[0], s[1]
            self._state.theta,  self._state.theta_dot  = s[2], s[3]
            ds = self.cartpole.get_dynamic(self._state, u)
            # Euler step
            s = s + ds * self.dt

            # incremental cost
            total += c0*(s[0]    )**2 \
                   + c1*(s[2]-np.pi)**2 \
                   + c2*(u       )**2
        return total

    def solve_mpc(self, current_state: State):
        # initial guess & bounds
        u0     = np.zeros(self.horizon)
        bounds = [self.u_lim]*self.horizon

        # wrap objective so scipy sees only one argument
        obj = lambda u: self._total_cost(u, current_state)

        res = minimize(obj, u0,
                       method='L-BFGS-B',
                       bounds=bounds,
                       options={'maxiter': self.num_iterations})
        print(f"final sequence: {res.x}")
        # return first control
        return float(res.x[0])


class SwingNMPCControllerSC:
    def __init__(self, cartpole: CartPole, horizon=10, dt=0.007, num_iterations=100, u_lim=(-0.7, 0.7)):
        self.cartpole = cartpole
        self.horizon  = horizon
        self.dt       = dt
        self.u_lim    = u_lim
        self.num_iterations = num_iterations
        self.costs   = [15, 6, 3, 0.7]  # [position, abgular_angle, potential, effort]

        # a single State instance we’ll overwrite each step
        self._state = State(timestamp=0, dt=dt, x=0, x_dot=0, theta=0, theta_dot=0, old_ctrl=0)

    def _total_cost(self, u_seq, init_state: State):
        s = np.array([init_state.x, init_state.x_dot, init_state.theta, init_state.theta_dot])
        total = 0.0
        c0, c1, c2, c3 = self.costs
        M, m, L, b_c, f_c, b_p, f_p, K_pf = self.cartpole.params

        for u in u_seq:
            # update our scratch-State and compute derivatives
            self._state.x,      self._state.x_dot      = s[0], s[1]
            self._state.theta,  self._state.theta_dot  = s[2], s[3]
            ds = self.cartpole.get_dynamic(self._state, u)
            # Euler step
            s = s + ds * self.dt

            # Update cost based on swinging up the pole
            position_energy = -(abs(s[0]))**2  # closer to the center - more energy

            kinetic_energy = 0.5 * m * L**2 * s[3]**2

            potential_energy = m * self.cartpole.g * L * (1 - np.cos(s[2]))

            effort_penalty = -u**2

            # Add the individual costs for each timestep
            total += c0*position_energy \
                   + c1*kinetic_energy \
                   + c2*potential_energy \
                   + c3*effort_penalty

        return 10 - total

    def solve_mpc(self, current_state: State):
        # initial guess & bounds
        u0     = np.zeros(self.horizon)
        bounds = [self.u_lim] * self.horizon

        # wrap objective so scipy sees only one argument
        obj = lambda u: self._total_cost(u, current_state)

        res = minimize(obj, u0,
                       method='L-BFGS-B',
                       bounds=bounds,
                       options={'maxiter': self.num_iterations})
        print(f"final sequence: {res.x}")
        
        # Return first control input (what to apply)
        return float(res.x[0])

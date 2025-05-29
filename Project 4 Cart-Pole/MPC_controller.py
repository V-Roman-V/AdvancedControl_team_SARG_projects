import casadi as ca
import numpy as np
from cart_pole import CartPole, State

class NMPCController:
    def __init__(self, cartpole: CartPole, horizon=50, dt=0.007):
        self.dt, self.N = dt, horizon
        self.model = cartpole

        # 1) build symbolic dynamics once
        self._build_casadi_dynamics()

        # 2) set up Opti
        self.opti = ca.Opti()
        nx, nu = 4, 1
        X = self.opti.variable(nx, self.N+1)
        U = self.opti.variable(nu, self.N)
        X0 = self.opti.parameter(nx)
        # bounds
        self.opti.subject_to(self.opti.bounded(-0.65, U, 0.65))
        # dynamics constraints
        for k in range(self.N):
            xk = X[:,k]; uk = U[:,k]
            x_next = xk + self.f_casadi(xk, uk)*self.dt
            self.opti.subject_to(X[:,k+1] == x_next)
        # init
        self.opti.subject_to(X[:,0] == X0)
        # cost (as before)…
        Q = np.diag([1,0.1,10,0.1]); R = 0.01
        cost = 0
        for k in range(self.N):
            dx = X[:,k] - ca.vertcat(0,0,ca.pi,0)
            cost += dx.T@Q@dx + R*(U[:,k]**2)
        cost += 100*(X[2,self.N]-ca.pi)**2
        self.opti.minimize(cost)
        self.opti.solver('ipopt', {"ipopt.max_iter":200, "ipopt.print_level":0})

        self.X, self.U, self.X0 = X, U, X0

    def _build_casadi_dynamics(self):
        # grab params and make CasADi constants
        p = self.model.params
        M, m, L = [ca.DM(p[i]) for i in (0,1,2)]
        b_c, f_c = ca.DM(p[3]), ca.DM(p[4])
        b_p, f_p = ca.DM(p[5]), ca.DM(p[6])
        K_pf = ca.DM(p[7])
        g = self.model.g

        x = ca.SX.sym('x',4); u = ca.SX.sym('u',1)
        # states: x[0]=x, x[1]=x_dot, x[2]=theta, x[3]=theta_dot
        s, c = ca.sin(x[2]), ca.cos(x[2])
        s_xdot, s_tdot = ca.sign(x[1]), ca.sign(x[3])

        F_fric = b_c*x[1] + f_c*s_xdot
        T_fric = b_p*x[3] + f_p*s_tdot
        u_f = K_pf*(u - x[1])

        D = M + m
        mlc = m*L*c
        alpha = m*L**2 - (mlc**2)/D
        beta = -m*g*L*s \
               - (mlc/D)*(-M*u_f + F_fric + m*L*(x[1]**2)*s) \
               - T_fric

        theta_dd = beta/alpha
        pole_force = mlc*theta_dd - m*L*(x[1]**2)*s
        x_dd = u_f + pole_force/100

        f = ca.vertcat(x[1], x_dd, x[3], theta_dd)
        self.f_casadi = ca.Function('f_casadi', [x,u], [f])

    def compute_control(self, state: State):
        x0 = np.array([state.x, state.x_dot, state.theta, state.theta_dot])
        self.opti.set_value(self.X0, x0)
        sol = self.opti.solve()
        return float(sol.value(self.U[:,0]))

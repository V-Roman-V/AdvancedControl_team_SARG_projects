import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import imageio
from cartpole import CartPoleParams

class CartPoleVisualizer:
    def __init__(self, mode='realtime', cartpole_params: CartPoleParams = None):
        self.mode = mode
        self.cartpole_params = cartpole_params if cartpole_params else CartPoleParams()
        size_x, size_y = 16, 8

        self.state_trace_x = []
        self.state_trace_u = []
        self.state_trace_theta = []
        self.state_trace_time = []
        self.frames = []

        if self.mode in ['realtime', 'gif']:
            self.fig, (self.ax_cp, self.ax_plot) = plt.subplots(2, 1, figsize=(size_x, size_y))
            self.ax_cp.set_xlim(-10, 10)
            self.ax_cp.set_ylim(-1.1, 1.4)
            self.ax_cp.set_aspect('equal', adjustable='box')
            self.ax_cp.grid(True)
            self.ax_cp.set_title('CartPole')

            self.ax_plot.set_title("Position, Theta and Control Input Over Time")
            self.ax_plot.set_xlabel("Time (s)")
            self.ax_plot.set_ylabel("x / theta")
            self.ax_plot_right = self.ax_plot.twinx()
            self.ax_plot_right.set_ylabel("Control (u)", color='g')
            self.ax_plot_right.tick_params(axis='y', labelcolor='g')

            self.line_x, = self.ax_plot.plot([], [], 'r-', label='x(t)')
            self.line_theta, = self.ax_plot.plot([], [], 'b-', label='theta(t)')
            self.line_u, = self.ax_plot_right.plot([], [], 'g-', label='u(t)')

            self.cart_patch = None
            self.pole_line = None

            if self.mode == 'realtime':
                plt.ion()
                plt.show()

        elif self.mode == 'time_plot':
            self.fig, (self.ax_pos, self.ax_theta, self.ax_u) = plt.subplots(3, 1, figsize=(size_x, size_y))
            self.ax_pos.set_title('Position (X) Over Time')
            self.ax_pos.set_xlabel('Time (s)')
            self.ax_pos.set_ylabel('Position (m)')
            self.ax_theta.set_title('Theta Over Time')
            self.ax_theta.set_xlabel('Time (s)')
            self.ax_theta.set_ylabel('Theta (radians)')
            self.ax_u.set_title('Control Input Over Time')
            self.ax_u.set_xlabel('Time (s)')
            self.ax_u.set_ylabel('Force (N)')

    def _draw_cart_pole(self, x, theta):
        cart_width = 0.4
        cart_height = 0.2
        pole_length = self.cartpole_params.l

        self.ax_cp.clear()
        self.ax_cp.set_xlim(-10, 10)
        self.ax_cp.set_ylim(-1.1, 1.4)
        self.ax_cp.set_aspect('equal', adjustable='box')
        self.ax_cp.grid(True)

        cart_patch = Rectangle((x - cart_width / 2, -cart_height / 2),
                               cart_width, cart_height, fc='gray', zorder=2)
        self.ax_cp.add_patch(cart_patch)

        pole_x = x + pole_length * np.sin(theta)
        pole_y = cart_height / 2 + pole_length * np.cos(theta)
        self.ax_cp.plot([x, pole_x], [cart_height / 2, pole_y], 'k-', lw=3)
        self.ax_cp.set_title(f"CartPole Phase Space: t = {self.state_trace_time[-1]:.2f}s")

    def update(self, state, control, t):
        x, _, theta, _ = state
        self.state_trace_x.append(x)
        self.state_trace_u.append(control)
        self.state_trace_theta.append(theta)
        self.state_trace_time.append(t)

        if self.mode in ['realtime', 'gif']:
            self._draw_cart_pole(x, theta)

            self.line_x.set_data(self.state_trace_time, self.state_trace_x)
            self.line_theta.set_data(self.state_trace_time, self.state_trace_theta)
            self.line_u.set_data(self.state_trace_time, self.state_trace_u)

            self.ax_plot.relim()
            self.ax_plot.autoscale_view()
            self.ax_plot.grid(True)

            self.ax_plot_right.relim()
            self.ax_plot_right.autoscale_view()

            lines1, labels1 = self.ax_plot.get_legend_handles_labels()
            lines2, labels2 = self.ax_plot_right.get_legend_handles_labels()
            self.ax_plot.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

            self.fig.tight_layout()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            if self.mode == 'gif':
                width, height = self.fig.canvas.get_width_height()
                image = np.frombuffer(self.fig.canvas.buffer_rgba(), dtype=np.uint8).reshape((height, width, 4))
                self.frames.append(image[..., :3].copy())

        elif self.mode == 'time_plot':
            self.ax_pos.plot(self.state_trace_time, self.state_trace_x, 'r-', lw=1)
            self.ax_theta.plot(self.state_trace_time, self.state_trace_theta, 'b-', lw=1)
            self.ax_u.plot(self.state_trace_time, self.state_trace_u, 'g-', lw=1)

            self.fig.tight_layout()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def finalize(self, save_path='./cartpole.gif'):
        if self.mode == 'final':
            plt.ioff()
            plt.show()
        elif self.mode == 'gif' and save_path:
            print("Saving CartPole gif...")
            imageio.mimsave(save_path, self.frames, fps=20)
            print(f"Saved gif to: {save_path}")
            self.fig.savefig('cartpole_time_plots.png')
            print("Saved time plots as 'cartpole_time_plots.png'")
        elif self.mode == 'time_plot':
            self.fig.tight_layout()
            self.fig.savefig('cartpole_time_plots.png')
            print("Saved time plots as 'cartpole_time_plots.png'")
            plt.show()

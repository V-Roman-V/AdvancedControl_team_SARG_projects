import numpy as np
import matplotlib.pyplot as plt
from cartpole import CartPoleParams
from matplotlib.patches import Rectangle
import imageio

class CartPoleVisualizer:
    def __init__(self, mode='realtime', cartpole_params: CartPoleParams = None):
        self.mode = mode
        self.cartpole_params = cartpole_params if cartpole_params else CartPoleParams()
        size_x, size_y = 16, 6
        self.fig, self.ax = plt.subplots(figsize=(size_x, size_y))
        self.frames = []

        self.cart_patch = None
        self.pole_line = None
        self.state_trace_x = []
        self.state_trace_height = []
        self.trace_line, = self.ax.plot([], [], 'b--', alpha=0.3, linewidth=1, zorder=0)

        self.ax.set_title('Cart-Pole Simulation')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Pole Tip Height (m)')
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')

        y_lim = self.cartpole_params.l * 2.5 # Allow some space above the pole tip
        x_lim = y_lim / size_y * size_x
        self.ax.set_xlim(-x_lim, x_lim)
        self.ax.set_ylim(-y_lim, y_lim)

        if self.mode == 'realtime':
            plt.ion()
            plt.show()

    def _draw_cart_pole(self, x, theta):
        cart_width = 0.4
        cart_height = 0.2
        pole_length = self.cartpole_params.l

        # Remove previous drawings
        if self.cart_patch:
            self.cart_patch.remove()
        if self.pole_line:
            self.pole_line.remove()

        # Draw cart
        self.cart_patch = Rectangle((x - cart_width / 2, -cart_height / 2),
                                    cart_width, cart_height,
                                    fc='gray', zorder=2)
        self.ax.add_patch(self.cart_patch)

        # Draw pole
        pole_x = x + pole_length * np.sin(theta)
        pole_y = cart_height / 2 + pole_length * np.cos(theta)
        self.pole_line, = self.ax.plot([x, pole_x], [cart_height / 2, pole_y],
                                       color='black', lw=3, zorder=3)

    def update(self, state, t):
        x, _, theta, _ = state
        self._draw_cart_pole(x, theta)

        # Store phase trajectory in (x, pole_tip_height)
        self.state_trace_x.append(x)
        tip_height = self.cartpole_params.l * np.cos(theta)
        self.state_trace_height.append(tip_height)
        self.trace_line.set_data(self.state_trace_x, self.state_trace_height)

        self.ax.set_title(f"Cart-Pole Phase Space: t = {t:.2f}s")

        if self.mode == 'realtime':
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        elif self.mode == 'gif':
            self.fig.canvas.draw()
            width, height = self.fig.canvas.get_width_height()
            image = np.frombuffer(self.fig.canvas.buffer_rgba(), dtype=np.uint8)
            image = image.reshape((height, width, 4))
            self.frames.append(image[..., :3].copy())  # Drop alpha

    def finalize(self, save_path='./cartpole.gif'):
        if self.mode == 'final':
            plt.ioff()
            plt.show()
        elif self.mode == 'gif' and save_path:
            print("Saving CartPole gif...")
            imageio.mimsave(save_path, self.frames, fps=20)
            print(f"Saved gif to: {save_path}")

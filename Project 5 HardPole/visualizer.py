import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import imageio

class CartPoleVisualizer:
    def __init__(self, mode='realtime'):
        self.mode = mode
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.frames = []

        self.cart_patch = None
        self.pole_line = None
        self.trajectory_line, = self.ax.plot([], [], 'b--', linewidth=1)
        self.x_history = []
        self.t_history = []

        self.ax.set_title('Cart-Pole Simulation')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Height (m)')
        self.ax.grid(True)
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-1.0, 1.5)

        if self.mode == 'realtime':
            plt.ion()
            plt.show()

    def _draw_cart_pole(self, x, theta):
        cart_width = 0.4
        cart_height = 0.2
        pole_length = 1.0

        # Clear previous
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

        # Trajectory plot
        self.t_history.append(t)
        self.x_history.append(x)
        self.trajectory_line.set_data(self.t_history, self.x_history)

        self.ax.set_title(f"Cart-Pole, t = {t:.2f}s")

        if self.mode == 'realtime':
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        elif self.mode == 'gif':
            self.fig.canvas.draw()
            # Use buffer_rgba() for Agg backend and convert RGBA to RGB
            width, height = self.fig.canvas.get_width_height()
            image = np.frombuffer(self.fig.canvas.buffer_rgba(), dtype=np.uint8)
            image = image.reshape((height, width, 4))
            image_rgb = image[..., :3].copy()  # Drop alpha channel
            self.frames.append(image_rgb)

    def finalize(self, save_path='./cartpole.gif'):
        if self.mode == 'final':
            plt.ioff()
            plt.show()
        elif self.mode == 'gif' and save_path:
            print("Saving CartPole gif...")
            imageio.mimsave(save_path, self.frames, fps=20)
            print(f"Saved gif to: {save_path}")

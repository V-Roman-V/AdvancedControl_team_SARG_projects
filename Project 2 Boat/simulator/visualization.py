import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
import imageio
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D

class BoatVisualizer:
    def __init__(self, mode='realtime', desired_trajectories=None, control_limits=None, boat_types=None, wind_velocity=(0,0)):
        self.mode = mode
        self.wind_velocity = np.array(wind_velocity)
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.boat_patches = []
        self.trajectory_lines = []
        self.desired_traj_lines = []
        self.frames = []
        self.desired_trajs = [np.array(dt) for dt in desired_trajectories]
        self.control_limits = control_limits
        self.boat_types = boat_types
        self.num_boats = len(boat_types)
        self.colors = plt.cm.tab20(np.linspace(0, 1, len(self.boat_types)))
        self.norms = {
            'differential': Normalize(0, control_limits['differential']),
            'steerable': Normalize(0, control_limits['steerable'][0])
        }
        self.cmap = plt.cm.Reds

        # Wind visualization parameters
        self.wind_dots = None
        self.num_wind_dots = 200
        self.wind_dot_positions = None
        self.wind_dot_size = 20
        self.wind_dot_alpha = 0.5
        self.plot_limits = None  # Will be set during first update

        self.ax.set_title('Multi-Boat Trajectory Tracking')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.grid(True)
        self.ax.axis('equal')
        self.ax.set_aspect('equal', adjustable='box')

        # Create legend handles for boat types
        type_to_color = {}
        for i, boat_type in enumerate(self.boat_types):
            if boat_type not in type_to_color:
                type_to_color[boat_type] = self.colors[i]

        self.boat_type_legend_handles = [
            Line2D([0], [0], color=type_to_color[btype], lw=4, label=f'{btype.capitalize()} Boat')
            for btype in type_to_color
        ]

        if self.mode == 'realtime':
            plt.ion()
            plt.show()

    def _create_boat_triangle(self, x, y, psi, size=0.3, color='blue'):
        points = np.array([[-size, -size], [size*2, 0], [-size, size]]).T
        rotation = np.array([[np.cos(psi), -np.sin(psi)],
                             [np.sin(psi),  np.cos(psi)]])
        rotated_points = (rotation @ points).T + [x, y]
        return Polygon(rotated_points, closed=True, color=color, alpha=0.8)

    def _initialize_wind_dots(self, x_lim, y_lim):
        """Initialize wind dots within the given plot limits"""
        self.wind_dot_positions = np.column_stack([
            np.random.uniform(x_lim[0], x_lim[1], self.num_wind_dots),
            np.random.uniform(y_lim[0], y_lim[1], self.num_wind_dots)
        ])
        self.wind_dots = self.ax.scatter(
            self.wind_dot_positions[:, 0],
            self.wind_dot_positions[:, 1],
            s=self.wind_dot_size,
            color='gray',
            alpha=self.wind_dot_alpha,
            marker='.',
            label='Wind'
        )

    def _update_wind_dots(self, x_lim, y_lim, dt):
        """Update wind dot positions and handle boundary conditions"""
        if self.wind_dots is None:
            return

        # Move dots with wind velocity
        self.wind_dot_positions += self.wind_velocity * dt

        # Check boundaries and reset dots that go out of bounds
        reach_x_max = self.wind_dot_positions[:, 0] > x_lim[1]
        reach_x_min = self.wind_dot_positions[:, 0] < x_lim[0]
        reach_y_max = self.wind_dot_positions[:, 1] > y_lim[1]
        reach_y_min = self.wind_dot_positions[:, 1] < y_lim[0]

        # reset dot coordinate
        self.wind_dot_positions[reach_x_max, 0] = x_lim[0]
        self.wind_dot_positions[reach_x_min, 0] = x_lim[1]
        self.wind_dot_positions[reach_y_max, 1] = y_lim[0]
        self.wind_dot_positions[reach_y_min, 1] = y_lim[1]

        # Update scatter plot data
        self.wind_dots.set_offsets(self.wind_dot_positions)

    def update(self, current_states, trajectories, current_step, controls, dt):
        trajectories = np.array(trajectories)

        # Clear previous boat patches
        for patch in self.boat_patches:
            patch.remove()
        self.boat_patches = []

        all_x = np.concatenate([traj[:, 0] for traj in trajectories])
        all_y = np.concatenate([traj[:, 1] for traj in trajectories])
        x_min, x_max = np.min(all_x), np.max(all_x)
        y_min, y_max = np.min(all_y), np.max(all_y)
        x_pad = (x_max - x_min) * 0.05
        y_pad = (y_max - y_min) * 0.05
        x_lim = (np.min(all_x) - x_pad, np.max(all_x) + x_pad)
        y_lim = (np.min(all_y) - y_pad, np.max(all_y) + y_pad)
        self.ax.set_xlim(x_lim)
        self.ax.set_ylim(y_lim)

        if self.wind_dots is None and np.any(self.wind_velocity != 0):
            self._initialize_wind_dots(x_lim, y_lim)

        self._update_wind_dots(x_lim, y_lim, dt=dt)

        # Update boats
        for i in range(self.num_boats):
            x, y, psi = current_states[i]
            boat_color = self.colors[i % len(self.colors)]
            boat_patch = self._create_boat_triangle(x, y, psi, color=boat_color)
            self.boat_patches.append(boat_patch)
            self.ax.add_patch(boat_patch)

        self._update_trajectories(trajectories)

        self.ax.set_title(f'Boat Trajectory Control, t = {current_step:.1f}s')
        if self.mode == 'realtime':
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            # plt.pause(0.00001)
        elif self.mode == 'gif':
            self.fig.canvas.draw()
            image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
            image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            self.frames.append(image)



    def _update_trajectories(self, trajectories):
        for i in range(self.num_boats):
            traj = np.array(trajectories[i])
            if len(self.trajectory_lines) <= i:
                traj_line, = self.ax.plot([], [], ':', color=self.colors[i],
                                          label=f'Boat {i+1} Path')
                self.trajectory_lines.append(traj_line)
            self.trajectory_lines[i].set_data(traj[:,0], traj[:,1])

        for i, desired_traj in enumerate(self.desired_trajs):
            if len(self.desired_traj_lines) <= i:
                desired_line, = self.ax.plot(
                    desired_traj[0], desired_traj[1], 'o',
                    color=self.colors[i], label=f'Boat {i+1} Desired'
                )
                self.desired_traj_lines.append(desired_line)

    def finalize(self, save_path='./gif/simulation.gif'):
        handles, labels = self.ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        handles, labels = self.ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))

        # Add boat type color legend
        all_handles = list(unique.values()) + self.boat_type_legend_handles
        all_labels = list(unique.keys()) + [h.get_label() for h in self.boat_type_legend_handles]

        self.ax.legend(all_handles, all_labels, loc='upper right')

        if self.mode == 'final':
            plt.ioff()
            plt.show()
        elif self.mode == 'gif' and save_path:
            print("Saving gif...")
            # print(self.frames)
            imageio.mimsave(save_path, self.frames, fps=10, loop=0)
            print(f"Gif `{save_path}` is saved")
    
    def create_target_phase_plot(self, trajectories, desired_states, save_path=None):
        """
        Create a phase plot with the distance to the target and the angle relative to the target for all boats.
        """
        fig, ax = plt.subplots(figsize=(10, 8))

        for i in range(self.num_boats):
            traj = np.array(trajectories[i])
            target_x, target_y = desired_states[i][0], desired_states[i][1]

            distances = []
            angles = []

            for state in traj:
                boat_x, boat_y = state[0], state[1]
                distance = np.sqrt((boat_x - target_x)**2 + (boat_y - target_y)**2)

                delta_x = target_x - boat_x
                delta_y = target_y - boat_y
                angle = np.arctan2(delta_y, delta_x) - state[2]
                angle = (angle + np.pi) % (2 * np.pi) - np.pi

                distances.append(distance)
                angles.append(angle)

            color = self.colors[i % len(self.colors)]
            ax.plot(distances, angles, label=f'Boat {i+1}', color=color, linewidth=2)

        ax.set_title('Phase Plot: Distance to Target vs Angle Relative to Target')
        ax.set_xlabel('Distance to Target (m)')
        ax.set_ylabel('Angle to Target (radians)')
        ax.axhline(0, color='black', linewidth=1)
        ax.axhline(np.pi, color='red', linewidth=1, linestyle='--')
        ax.grid(True)
        ax.legend(loc='upper right')

        if save_path:
            fig.savefig(save_path)
            print(f"Phase plot saved at {save_path}")
        
        plt.show()

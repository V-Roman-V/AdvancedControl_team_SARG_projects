import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
import imageio
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D
from wind_generator import WindField

class BoatVisualizer:
    def __init__(self, mode='realtime', desired_trajectories=None, control_limits=None, boat_types=None, wind_field: WindField = None):
        self.mode = mode
        self.wind_field : WindField = wind_field
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
        self.type_to_color = {
            'differential': 'red', 
            'steerable': 'blue'
        }

        self.boat_type_legend_handles = [
            Line2D([0], [0], color=self.type_to_color[btype], lw=4, label=f'{btype.capitalize()} Boat')
            for btype in self.type_to_color
        ]

        if self.mode == 'realtime':
            plt.ion()
            plt.show()

    def _create_boat_triangle(self, x, y, psi, size=0.3, color='blue'):
        points = np.array([[-size, -size], [size*2, 0], [-size, size]]).T
        rotation = np.array([[np.cos(psi), -np.sin(psi)],
                             [np.sin(psi),  np.cos(psi)]])
        rotated_points = (rotation @ points).T + [x, y]
        return Polygon(rotated_points, closed=True, color=color, alpha=0.5, zorder=3)

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
            label='Wind',
            zorder=0  # Add this line
        )

    def _update_wind_dots(self, x_lim, y_lim, dt):
        """Update wind dot positions and handle boundary conditions"""
        if self.wind_dots is None:
            return

        # Move dots with wind velocity
        for i in range(len(self.wind_dot_positions)):
            dot_pos =self.wind_dot_positions[i]
            wind = self.wind_field.get_wind([dot_pos[0], dot_pos[1]])
            self.wind_dot_positions[i, 0] += wind[0] * dt
            self.wind_dot_positions[i, 1] += wind[1] * dt

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

        if self.wind_dots is None and self.wind_field is not None:
            self._initialize_wind_dots(x_lim, y_lim)

        self._update_wind_dots(x_lim, y_lim, dt=dt)

        # Add boat type color legend
        if self.wind_dots is not None:
            all_handles = self.boat_type_legend_handles + list([self.wind_dots])
            all_labels = [h.get_label() for h in self.boat_type_legend_handles] + ['wind']
        else:
            all_handles = self.boat_type_legend_handles
            all_labels = [h.get_label() for h in self.boat_type_legend_handles] 
        self.ax.legend(all_handles, all_labels, loc='upper right')

        # Update boats
        for i in range(self.num_boats):
            x, y, psi = current_states[i]
            boat_color = self.type_to_color[self.boat_types[i]]
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
                traj_line, = self.ax.plot([], [], ':', color=self.type_to_color[self.boat_types[i]], label=f'Boat {i+1} Path',zorder=1)
                self.trajectory_lines.append(traj_line)
            self.trajectory_lines[i].set_data(traj[:,0], traj[:,1])

        for i, desired_traj in enumerate(self.desired_trajs):
            if len(self.desired_traj_lines) <= i:
                desired_line, = self.ax.plot(desired_traj[0], desired_traj[1], 'o', color=self.type_to_color[self.boat_types[i]], label=f'Boat {i+1} Desired', zorder=2)
                self.desired_traj_lines.append(desired_line)

    def finalize(self, save_path='./gif/simulation.gif'):
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
        
        # plt.show()

    def create_estimated_wind_plot(self, trajectories, save_path=None):
        """Create plots showing wind velocity estimates for all boats over time.
        
        Args:
            trajectories: List of boat trajectories, where each trajectory is 
                        a list of BoatState objects
            save_path: Optional path to save the figure
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, num=3, figsize=(10, 10))
        
        # Create time axis based on number of steps
        num_steps = len(trajectories[0])
        time = np.arange(num_steps)  # Using step indices as time
        
        # Plot estimates for each boat
        for i in [1]:
            boat_traj = trajectories[i]
            # Extract wind estimates from boat states
            wx_estimates = [state[6] for state in boat_traj]
            wy_estimates = [state[7] for state in boat_traj]
            
            true_x_wind = [self.wind_field.get_wind([s[0], s[1]])[0] for s in boat_traj]
            true_y_wind = [self.wind_field.get_wind([s[0], s[1]])[1] for s in boat_traj]

            color = self.colors[i % len(self.colors)]
            label = f'Boat {i+1} ({self.boat_types[i]})'
            
            ax1.plot(time, wx_estimates, color=color, label=label)
            ax2.plot(time, wy_estimates, color=color, label=label)

            ax1.plot(time, true_x_wind, color='black', linestyle='--', linewidth=2, label='True Wind X')
            ax2.plot(time, true_y_wind, color='black', linestyle='--', linewidth=2, label='True Wind Y')

        # Configure plots
        ax1.set_title('Wind X Component Estimates')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.grid(True)
        ax1.legend(loc='upper right')
        
        ax2.set_title('Wind Y Component Estimates')
        ax2.set_xlabel('Time Step')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.grid(True)
        ax2.legend(loc='upper right')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path)
            print(f"Wind estimation plots saved to {save_path}")
        
        # plt.show()
    def create_control_plot(self, control_histories, boat_types, save_path=None):
        """Create separate control input plots for differential and steerable boats.
        
        Args:
            control_histories: List of control input histories, where each history is a list of (u1, u2) or (uf, us) tuples.
            boat_types: List of boat types for each boat ('differential' or 'steerable').
            save_path: Optional path to save the figure
        """
        # Separate control histories by type
        diff_indices = [i for i, t in enumerate(boat_types) if t == 'differential']
        steer_indices = [i for i, t in enumerate(boat_types) if t == 'steerable']
    
        # Create figure for differential boats if any
        if diff_indices:
            fig_diff, (ax1_diff, ax2_diff) = plt.subplots(2, 1, num=4, figsize=(10, 10))
            num_steps = len(control_histories[diff_indices[0]])
            time = np.arange(num_steps)
    
            for i in diff_indices:
                controls = control_histories[i]
                u1 = [c[0] for c in controls]
                u2 = [c[1] for c in controls]
                color = self.colors[i % len(self.colors)]
                label = f'Boat {i+1} (Differential)'
    
                ax1_diff.plot(time, u1, color=color, label=label)
                ax2_diff.plot(time, u2, color=color, label=label)
    
            ax1_diff.set_title('Differential Boat: Left Thrust (u1) over Time')
            ax1_diff.set_ylabel('Thrust (N)')
            ax1_diff.grid(True)
            ax1_diff.legend(loc='upper right')
    
            ax2_diff.set_title('Differential Boat: Right Thrust (u2) over Time')
            ax2_diff.set_xlabel('Time Step')
            ax2_diff.set_ylabel('Thrust (N)')
            ax2_diff.grid(True)
            ax2_diff.legend(loc='upper right')
    
            plt.tight_layout()
    
            if save_path:
                diff_path = save_path.replace('.png', '_differential.png')
                fig_diff.savefig(diff_path)
                print(f"Differential control plot saved to {diff_path}")
    
        # Create figure for steerable boats if any
        if steer_indices:
            fig_steer, (ax1_steer, ax2_steer) = plt.subplots(2, 1, num=5, figsize=(10, 10))
            num_steps = len(control_histories[steer_indices[0]])
            time = np.arange(num_steps)
    
            for i in steer_indices:
                controls = control_histories[i]
                uf = [c[0] for c in controls]
                us = [c[1] for c in controls]
                color = self.colors[i % len(self.colors)]
                label = f'Boat {i+1} (Steerable)'
    
                ax1_steer.plot(time, uf, color=color, label=label)
                ax2_steer.plot(time, us, color=color, label=label)
    
            ax1_steer.set_title('Steerable Boat: Forward Thrust (uf) over Time')
            ax1_steer.set_ylabel('Thrust (N)')
            ax1_steer.grid(True)
            ax1_steer.legend(loc='upper right')
    
            ax2_steer.set_title('Steerable Boat: Steering Angle (us) over Time')
            ax2_steer.set_xlabel('Time Step')
            ax2_steer.set_ylabel('Angle (rad)')
            ax2_steer.grid(True)
            ax2_steer.legend(loc='upper right')
    
            plt.tight_layout()
    
            if save_path:
                steer_path = save_path.replace('.png', '_steerable.png')
                fig_steer.savefig(steer_path)
                print(f"Steerable control plot saved to {steer_path}")
    
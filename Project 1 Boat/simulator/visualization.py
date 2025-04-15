import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
import imageio
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize


class BoatVisualizer:
    def __init__(self, mode='realtime', desired_trajectories=None, control_limits=None, num_boats=1):
        """
        Initialize boat trajectory visualizer.
        
        Modes:
        - 'realtime': Update visualization at each timestep
        - 'final': Show only final result
        - 'gif': Save animation as GIF
        """
        self.mode = mode
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.boat_patches = []
        self.trajectory_lines = []
        self.desired_traj_lines = []
        self.thruster_patches = []
        self.frames = []
        self.desired_trajs = [np.array(dt) for dt in desired_trajectories] if desired_trajectories else []
        self.control_limits = control_limits
        self.num_boats = num_boats
        self.colors = plt.cm.tab10.colors  # Different colors for different boats

        # Store normalization for each boat
        self.norms = [Normalize(vmin=0, vmax=cl) for cl in control_limits]
        self.cmap = plt.cm.Reds

        # Initialize plot
        self.ax.set_title('Multi-Boat Trajectory Tracking')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.grid(True)
        self.ax.axis('equal')
        self.ax.set_aspect('equal', adjustable='box')

        # Add colorbars for each boat
        for i in range(num_boats):
            sm = ScalarMappable(norm=self.norms[i], cmap=self.cmap)
            self.fig.colorbar(sm, ax=self.ax, label=f'Boat {i+1} Thrust (N)')

        if self.mode == 'realtime':
            plt.ion()
            plt.show()

    def _create_boat_triangle(self, x, y, psi, controls, size=0.5, color='blue', control_limit=20):
        """Create triangle patch representing the boat with thrusters"""
        # Main boat triangle
        points = np.array([[-size, -size], [size*2, 0], [-size, size]]).T
        rotation = np.array([[np.cos(psi), -np.sin(psi)],
                            [np.sin(psi), np.cos(psi)]])
        rotated_points = np.dot(rotation, points).T + [x, y]
        
        # Thruster positions (at the back of the boat)
        offset = size * 1.2
        left_thruster_pos = np.dot(rotation, [-offset, -size*0.7]) + [x, y]
        right_thruster_pos = np.dot(rotation, [-offset, size*0.7]) + [x, y]
        
        # Create thrusters with boat-specific control limits
        left_thruster_points = self.__get_thruster(left_thruster_pos, rotation, controls[0], size, control_limit)
        right_thruster_points = self.__get_thruster(right_thruster_pos, rotation, controls[1], size, control_limit)

        return (
            Polygon(rotated_points, closed=True, color=color, alpha=0.8),
            Polygon(left_thruster_points, closed=True, color=self.cmap(self.norms[0](controls[0]))),
            Polygon(right_thruster_points, closed=True, color=self.cmap(self.norms[0](controls[1]))),
        )

    def __get_thruster(self, pos, rotation, control, size, control_limit):
        """Helper method to create thruster geometry"""
        width_size = size * 0.35
        length_size = abs(control) / control_limit * size * 1
        
        # Fixed array broadcasting by using proper shape
        scale = np.array([length_size, width_size])
        points = np.array([[0,-1], [-1,-1], [-1,1], [0, 1]]) * scale
        
        rotated_points = np.dot(rotation, points.T).T + pos
        return rotated_points

    def update(self, current_states, current_step, controls):
        """Update visualization with current states and controls"""
        # Clear previous boats
        for patch in self.boat_patches:
            patch.remove()
        for patch in self.thruster_patches:
            patch.remove()
        self.boat_patches = []
        self.thruster_patches = []

        # Update all boats
        for i in range(self.num_boats):
            # Ensure we have 3 values (x, y, psi)
            state = current_states[i]
            if len(state) >= 3:  # Ensure state includes x, y, psi
                x, y, psi = state[:3]
                boat_color = self.colors[i % len(self.colors)]
                
                # Create boat with individual control limit
                boat_patch, left_t, right_t = self._create_boat_triangle(
                    x, y, psi, controls[i], 
                    size=0.5, 
                    color=boat_color,
                    control_limit=self.control_limits[i]
                )
                
                self.boat_patches.append(boat_patch)
                self.thruster_patches.extend([left_t, right_t])
                
                self.ax.add_patch(boat_patch)
                self.ax.add_patch(left_t)
                self.ax.add_patch(right_t)

        # Update trajectories
        self._update_trajectories()
        
        # Handle visualization updates
        self.ax.set_title(f'Boat Trajectory Control, t = {current_step:.1f}s')
        if self.mode == 'realtime':
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
        elif self.mode == 'gif':
            self.fig.canvas.draw()
            image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
            image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            self.frames.append(image)

    def _update_trajectories(self):
        """Update trajectory lines for all boats"""
        # Actual trajectories
        for i in range(self.num_boats):
            if len(self.trajectory_lines) <= i:
                traj_line, = self.ax.plot([], [], color=self.colors[i], 
                                    label=f'Boat {i+1} Path')
                self.trajectory_lines.append(traj_line)

            if len(self.desired_trajs[i]) > 0:
                traj = self.desired_trajs[i]
                if traj.ndim == 1:
                    traj = traj.reshape(-1, 2)  # Ensure it's 2D, reshaping if necessary
                self.trajectory_lines[i].set_data(traj[:, 0], traj[:, 1])

        # Desired trajectories
        for i in range(len(self.desired_trajs)):
            if len(self.desired_traj_lines) <= i:
                desired_line, = self.ax.plot(
                    self.desired_trajs[i], self.desired_trajs[i],
                    '--', color=self.colors[i], marker='o', markersize=4,
                    label=f'Boat {i+1} Desired'
                )
                self.desired_traj_lines.append(desired_line)

    def finalize(self, trajectories, save_path=None):
        """Finalize visualization and save if needed"""
        # Add legend with proper labels
        handles, labels = self.ax.get_legend_handles_labels()
        unique_labels = []
        unique_handles = []
        for handle, label in zip(handles, labels):
            if label not in unique_labels:
                unique_labels.append(label)
                unique_handles.append(handle)
        self.ax.legend(unique_handles, unique_labels, loc='upper right')

        # Handle different visualization modes
        if self.mode == 'final':
            plt.ioff()
            plt.show()
        elif self.mode == 'gif' and save_path:
            print("Saving gif...")
            imageio.mimsave(save_path, self.frames, fps=20)
            print(f"Gif `{save_path}` is saved")

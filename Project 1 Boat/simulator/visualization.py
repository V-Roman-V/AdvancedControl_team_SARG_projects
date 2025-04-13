import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
import imageio
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize


class BoatVisualizer:
    def __init__(self, mode='realtime', desired_trajectory=None, control_limit=10):
        """
        Initialize boat trajectory visualizer.
        
        Modes:
        - 'realtime': Update visualization at each timestep
        - 'final': Show only final result
        - 'gif': Save animation as GIF
        """
        self.mode = mode
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.boat_patch = None
        self.trajectory_line = None
        self.desired_traj_line = None
        self.thruster_patches = []
        self.frames = []
        self.desired_traj = np.array(desired_trajectory)
        self.control_limit = control_limit
        
        # Initialize plot
        self.ax.set_title('Boat Trajectory Tracking')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.grid(True)
        self.ax.axis('equal')
        self.ax.set_aspect('equal', adjustable='box')  # Set equal aspect ratio
        
        # Add colorbar for thrust visualization
        self.norm = Normalize(vmin=-control_limit, vmax=control_limit)
        self.cmap = plt.cm.coolwarm
        self.sm = ScalarMappable(norm=self.norm, cmap=self.cmap)
        self.fig.colorbar(self.sm, ax=self.ax, label='Thruster Force (N)')

        if self.mode == 'realtime':
            plt.ion()
            plt.show()

    def _create_boat_triangle(self, x, y, psi, controls, size = 0.5):
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
        left_thruster_points = self.__get_thruster(left_thruster_pos, rotation, controls[0], size)
        right_thruster_points = self.__get_thruster(right_thruster_pos, rotation, controls[1], size)

        return (
            Polygon(rotated_points, closed=True, color='blue', alpha=0.8),
            Polygon(left_thruster_points, closed=True, color=self.cmap(self.norm(controls[0])), alpha=0.8),
            Polygon(right_thruster_points, closed=True, color=self.cmap(self.norm(controls[1])), alpha=0.8),
        )

    def __get_thruster(self, pos, rotation, control, size):
        width_size = size * 0.35
        length_size = abs(control) / self.control_limit * size * 1
        points = np.array([[0,-1], [-1,-1], [-1,1], [0, 1]]) * np.array([length_size, width_size])
        rotated_points = np.dot(rotation, points.T).T + pos
        return rotated_points

    def update(self, trajectory, current_step, controls):
        """Update visualization with current state and controls"""
        # Clear previous boat and thrusters
        if self.boat_patch:
            self.boat_patch.remove()
        for patch in self.thruster_patches:
            patch.remove()
        self.thruster_patches = []
            
        # Get current state
        x, y, psi = trajectory[-1][:3]
        
        # Auto-adjust boat size based on trajectory
        x_min = min(np.min(np.array(trajectory)[:,0]), np.min(self.desired_traj[:,0]))
        x_max = max(np.max(np.array(trajectory)[:,0]), np.max(self.desired_traj[:,0]))
        y_min = min(np.min(np.array(trajectory)[:,1]), np.min(self.desired_traj[:,1]))
        y_max = max(np.max(np.array(trajectory)[:,1]), np.max(self.desired_traj[:,1]))
        delta_max = max(y_max - y_min, x_max - x_min)
        boat_size = delta_max / 50

        # Create new boat and thrusters
        boat_patch, left_t_patch, right_t_patch = self._create_boat_triangle(x, y, psi, controls, size=boat_size)
        self.boat_patch = boat_patch
        self.thruster_patches.append(left_t_patch)
        self.thruster_patches.append(right_t_patch)
        self.ax.add_patch(self.boat_patch)
        self.ax.add_patch(left_t_patch)
        self.ax.add_patch(right_t_patch)

        # update title
        self.ax.set_title(f'Boat Trajectory Control, t = {current_step}')
        
        # Update trajectory line
        if not self.trajectory_line:
            traj = np.array(trajectory)
            self.trajectory_line, = self.ax.plot(traj[:,0], traj[:,1], 'b-', label='Actual Path')
        else:
            self.trajectory_line.set_data(np.array(trajectory)[:,:2].T)
            
        # Update desired trajectory
        if self.desired_traj is not None and not self.desired_traj_line:
            desired_points = np.array(self.desired_traj)
            self.desired_traj_line, = self.ax.plot(
                desired_points[:,0], desired_points[:,1], 
                'r--', marker='o', markersize=4, label='Desired Path'
            )
            
        # Handle different visualization modes
        if self.mode == 'realtime':
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
        elif self.mode == 'gif':
            self.fig.canvas.draw()
            image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
            image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            self.frames.append(image)

    def finalize(self, trajectory, save_path=None):
        """Finalize visualization and save if needed"""
        if self.mode == 'final':
            # Plot final result
            traj = np.array(trajectory)
            self.ax.plot(traj[:,0], traj[:,1], 'b-', label='Actual Path')
            
            if self.desired_traj is not None:
                desired_points = np.array(self.desired_traj)
                self.ax.plot(
                    desired_points[:,0], desired_points[:,1], 
                    'r--', marker='o', markersize=4, label='Desired Path'
                )
            
            self.ax.legend()
            plt.show()
            
        elif self.mode == 'gif' and save_path:
            # Save animation as GIF
            print("Saving gif...")
            imageio.mimsave(save_path, self.frames, fps=20)
            print(f"Gif `{save_path}` is saved")
            
        if self.mode == 'final':
            plt.ioff()
            plt.show()
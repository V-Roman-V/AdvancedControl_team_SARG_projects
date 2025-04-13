import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.lines import Line2D
import imageio

class BoatVisualizer:
    def __init__(self, mode='realtime', desired_trajectory=None):
        """
        Initialize boat trajectory visualizer.
        
        Modes:
        - 'realtime': Update visualization at each timestep
        - 'final': Show only final result
        - 'gif': Save animation as GIF
        """
        self.mode = mode
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.boat_patch = None
        self.trajectory_line = None
        self.desired_traj_line = None
        self.frames = []
        self.desired_traj = np.array(desired_trajectory)
        
        # Initialize plot
        self.ax.set_title('Boat Trajectory Tracking')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.grid(True)
        self.ax.axis('equal')

        if self.mode == 'realtime':
            plt.ion()
            plt.show()

    def _create_boat_triangle(self, x, y, psi, size = 0.5):
        """Create triangle patch representing the boat"""
        points = np.array([[-size, -size], [size*2, 0], [-size, size]]).T
        rotation = np.array([[np.cos(psi), -np.sin(psi)],
                            [np.sin(psi), np.cos(psi)]])
        rotated_points = np.dot(rotation, points).T + [x, y]
        return Polygon(rotated_points, closed=True, color='blue', alpha=0.8)

    def update(self, trajectory, current_step):
        """Update visualization with current state"""
        # Clear previous boat position
        if self.boat_patch:
            self.boat_patch.remove()
            
        # Get current state
        x, y, psi = trajectory[-1][:3]
        
        # Create new boat patch
        x_min = min(np.min(np.array(trajectory)[:,0]), np.min(self.desired_traj[:,0]))
        x_max = max(np.max(np.array(trajectory)[:,0]), np.max(self.desired_traj[:,0]))
        y_min = min(np.min(np.array(trajectory)[:,1]), np.min(self.desired_traj[:,1]))
        y_max = max(np.max(np.array(trajectory)[:,1]), np.max(self.desired_traj[:,1]))
        delta_max = max(y_max - y_min, x_max - x_min)
        boat_size = delta_max / 50
        self.boat_patch = self._create_boat_triangle(x, y, psi, size=boat_size)
        self.ax.add_patch(self.boat_patch)

        # update title
        self.ax.set_title(f'Boat Trajectory Tracking, t = {current_step}')
        
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
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
import imageio
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize

class BoatVisualizer:
    def __init__(self, mode='realtime', desired_trajectory=None, control_limit=10):
        self.mode = mode
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.boat_patch = None
        self.trajectory_line = None
        self.desired_traj_line = None
        self.thruster_patches = []
        self.frames = []
        self.desired_traj = np.array(desired_trajectory)
        self.control_limit = control_limit

        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')

        self.norm = Normalize(vmin=-control_limit, vmax=control_limit)
        self.cmap = plt.cm.coolwarm
        self.sm = ScalarMappable(norm=self.norm, cmap=self.cmap)
        self.fig.colorbar(self.sm, ax=self.ax, label='Thruster Force (N)')

        if self.mode == 'realtime':
            plt.ion()
            plt.show()

    def _create_boat_triangle(self, x, y, psi, controls, size=0.5):
        points = np.array([[-size, -size], [size*2, 0], [-size, size]]).T
        rotation = np.array([[np.cos(psi), -np.sin(psi)],
                             [np.sin(psi), np.cos(psi)]])
        rotated_points = (rotation @ points).T + [x, y]

        offset = size * 1.2
        left_thruster_pos = rotation @ [-offset, -size*0.7] + [x, y]
        right_thruster_pos = rotation @ [-offset, size*0.7] + [x, y]

        left_thruster = self._get_thruster(left_thruster_pos, rotation, controls[0], size)
        right_thruster = self._get_thruster(right_thruster_pos, rotation, controls[1], size)

        return (
            Polygon(rotated_points, color='blue', alpha=0.8),
            Polygon(left_thruster, color=self.cmap(self.norm(controls[0])), alpha=0.8),
            Polygon(right_thruster, color=self.cmap(self.norm(controls[1])), alpha=0.8)
        )

    def _get_thruster(self, pos, rotation, control, size):
        width = size * 0.35
        length = abs(control) / self.control_limit * size
        points = np.array([[0,-1], [-1,-1], [-1,1], [0,1]]) * [length, width]
        return (rotation @ points.T).T + pos

    def update(self, trajectory, current_step, controls):
        if self.boat_patch:
            self.boat_patch.remove()
        for patch in self.thruster_patches:
            patch.remove()
        self.thruster_patches = []

        traj_arr = np.array(trajectory)
        x, y, psi = traj_arr[-1][:3]

        boat_patch, left_thruster, right_thruster = self._create_boat_triangle(x, y, psi, controls, size=self._calculate_boat_size(traj_arr))
        self.boat_patch = boat_patch
        self.thruster_patches.extend([left_thruster, right_thruster])
        self.ax.add_patch(boat_patch)
        self.ax.add_patch(left_thruster)
        self.ax.add_patch(right_thruster)

        if not self.trajectory_line:
            self.trajectory_line, = self.ax.plot(traj_arr[:,0], traj_arr[:,1], 'b-', label='Actual Path')
        else:
            self.trajectory_line.set_data(traj_arr[:,0], traj_arr[:,1])

        if self.desired_traj is not None and not self.desired_traj_line:
            self.desired_traj_line, = self.ax.plot(
                self.desired_traj[:,0], self.desired_traj[:,1], 
                'r--o', markersize=4, label='Desired Path'
            )

        self.ax.legend(loc='upper right')
        self._adjust_view(traj_arr, padding=0.1)

        self.ax.set_title(f'Boat Trajectory Tracking (t = {current_step})')

        if self.mode == 'realtime':
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
        elif self.mode == 'gif':
            self.fig.canvas.draw()
            image = np.frombuffer(self.fig.canvas.tostring_argb(), dtype=np.uint8)
            image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (4,))
            image = image[:, :, 1:]
            self.frames.append(image)

    def _calculate_boat_size(self, trajectory):
        traj_span = np.ptp(trajectory[:, :2], axis=0).max()
        desired_span = np.ptp(self.desired_traj[:, :2], axis=0).max() if self.desired_traj is not None else traj_span
        return max(traj_span, desired_span) / 40

    def _adjust_view(self, trajectory, padding=0.1):
        traj_points = trajectory[:, :2]
        points = traj_points
        if self.desired_traj is not None:
            points = np.vstack([points, self.desired_traj[:, :2]])

        x_min, y_min = points.min(axis=0) - padding
        x_max, y_max = points.max(axis=0) + padding
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)

    def finalize(self, trajectory, save_path=None):
        if self.mode == 'final':
            self.update(trajectory, current_step='Final', controls=[0,0])
            plt.ioff()
            plt.show()

        elif self.mode == 'gif' and save_path:
            print("Saving gif...")
            imageio.mimsave(save_path, self.frames, fps=20)
            print(f"Gif `{save_path}` saved successfully.")

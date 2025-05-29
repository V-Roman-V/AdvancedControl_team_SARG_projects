

# Compare with PD/PID

Just PD for pole:
![tmp](gif/cartpole_pd_down.gif)
![tmp](gif/cartpole_pd_right.gif)

> Can flip and hold pole UP.

Added integral + window for theha and it can hold the position also!

![pid](gif/cartpole_pid.gif)

From the bottom:

![alt text](gif/cartpole_pid_down.gif)

![alt text](gif/cartpole_pid_bang_bang.gif)

Tuning pd add Integral part to the position and tune koeficient we got realy good result:

![alt text](gif/cartpole_pose_pid.gif)

# Compare with energy

Up the Pole

![alt text](gif/cartpole_energy.gif)

# Compare with hybrid (energy + pid)

![alt text](gif/cartpole_hynrid.gif)


# Compare MPC_Simple (in descrete form)

![alt text](gif/cartpole_mpc_simple.gif)
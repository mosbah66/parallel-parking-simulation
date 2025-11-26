"""
===============================================================================
Parallel Parking Simulation - IU Case Study (DLBROESR01_E – Mobile Robotics)
Author : Mosbah Alaa Mohammad Alalawi
Date   : 2025-11
Python : ≥ 3.9
===============================================================================
Description:
------------
This script models and simulates an *autonomous reverse parallel parking maneuver*
for a car-like mobile robot using the **kinematic bicycle model**.
It demonstrates nonholonomic motion, trajectory generation, and plausibility plots
to support the IU case study Task 1.

Model Equations:
----------------
x_dot = v * cos(theta)
y_dot = v * sin(theta)
theta_dot = v/L * tan(delta)

where:
x, y     = position of the rear axle center [m]
theta    = heading angle [rad]
v        = linear velocity [m/s]
delta    = steering angle [rad]
L        = wheelbase [m]

Outputs:
--------
- XY trajectory plot showing the car’s reverse-in path
- Steering angle and velocity profiles over time
- Animated visualization of poses along the trajectory
- Data suitable for report figures
===============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt

# -------------------------- Simulation Parameters -------------------------- #
L = 2.7            # Wheelbase (m)
car_length = 4.5   # Total car length (m)
car_width = 2.0    # Car width (m)
v_max = 1.0        # Maximum forward speed (m/s)
v_min = -1.0       # Maximum reverse speed (m/s)
dt = 0.05          # Time step (s)
t_end = 20         # Total simulation time (s)
time = np.arange(0, t_end, dt)

# Steering constraints
delta_max = np.radians(30)   # 30 degrees max steering angle

# -------------------------- Trajectory Definition -------------------------- #
# For a smooth reverse parallel park, we create a sinusoidal steering pattern.
# The car reverses into a slot using a controlled steering sequence.

v = np.linspace(0, v_min, len(time))  # gradually reverse
delta = delta_max * np.sin(2 * np.pi * time / t_end)  # smooth oscillating steering

# Initial position (start beside parking spot)
x, y, theta = [0.0], [0.0], [0.0]

# -------------------------- Simulation Loop (Forward Kinematics) ----------- #
for i in range(1, len(time)):
    x_dot = v[i] * np.cos(theta[i-1])
    y_dot = v[i] * np.sin(theta[i-1])
    theta_dot = (v[i] / L) * np.tan(delta[i])

    x_new = x[i-1] + x_dot * dt
    y_new = y[i-1] + y_dot * dt
    theta_new = theta[i-1] + theta_dot * dt

    x.append(x_new)
    y.append(y_new)
    theta.append(theta_new)

x, y, theta = np.array(x), np.array(y), np.array(theta)

# -------------------------- Visualization --------------------------------- #
plt.figure(figsize=(8,6))
plt.plot(x, y, label='Car Trajectory', linewidth=2)
plt.quiver(x[::40], y[::40], np.cos(theta[::40]), np.sin(theta[::40]),
           scale=20, width=0.005, color='r', label='Heading direction')
plt.title("Parallel Parking Maneuver (Reverse-In)")
plt.xlabel("X position [m]")
plt.ylabel("Y position [m]")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig("trajectory_plot.png", dpi=300)
plt.show()

# -------------------------- Steering & Velocity Profiles ------------------ #
fig, axs = plt.subplots(2, 1, figsize=(8,6), sharex=True)
axs[0].plot(time, np.degrees(delta), 'b', linewidth=2)
axs[0].set_ylabel("Steering Angle δ [deg]")
axs[0].grid(True)

axs[1].plot(time, v, 'r', linewidth=2)
axs[1].set_xlabel("Time [s]")
axs[1].set_ylabel("Velocity v [m/s]")
axs[1].grid(True)

plt.suptitle("Control Inputs vs Time")
plt.savefig("steering_velocity_profiles.png", dpi=300)
plt.show()

# -------------------------- End of Simulation ----------------------------- #
print("Simulation complete. Plots saved as 'trajectory_plot.png' and 'steering_velocity_profiles.png'.")

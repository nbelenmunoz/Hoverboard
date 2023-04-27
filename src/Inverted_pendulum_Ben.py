#inverted Pendulum
import numpy as np
import matplotlib.pyplot as plt
import control as ct
import control.matlab as cm

# System parameters
m = 0.1     # Mass of the pendulum [kg]
l = 0.5     # Length of the pendulum [m]
M = 0.5     # Mass of the reaction wheel [kg]
R = 0.1     # Radius of the reaction wheel [m]
g = 9.81    # Gravitational acceleration [m/s^2]
J = 1/3 * m * l**2 + 1/2 * M * R**2  # Inertia of the system

# State space representation of the system
A = np.array([[0, 1, 0, 0],
              [3*m*g/(2*J), 0, 0, 1],
              [0, 0, 0, 1],
              [-(m*g*l)/(J), 0, 0, 0]])

B = np.array([[0],
              [-3/(2*J)],
              [0],
              [1/J]])

C = np.array([[1, 0, 0, 0],
              [0, 0, 1, 0]])

D = np.zeros((2, 1))

# Create state-space model
sys = ct.StateSpace(A, B, C, D)

# Design a linear-quadratic regulator (LQR) controller
Q = np.diag([100, 1, 10, 1])
R = np.array([[1]])
K, _, _ = ct.lqr(sys, Q, R)

# Closed-loop system
sys_cl = ct.StateSpace(A - B @ K, B, C, D)

# Initial conditions (pendulum starts at a 10-degree angle)
x0 = np.array([np.deg2rad(10), 0, 0, 0])

# Simulate the system
t = np.linspace(0, 10, 1000) # 10 seconds, 1000 samples
_, y = ct.initial_response(sys_cl, t, x0)

# Plot the results
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t, np.rad2deg(y[0, :]))
plt.xlabel('Time [s]')
plt.ylabel('Pendulum angle [deg]')

plt.subplot(2, 1, 2)
plt.plot(t, np.rad2deg(y[1, :]))
plt.xlabel('Time [s]')
plt.ylabel('Reaction wheel angle [deg]')

plt.tight_layout()
plt.show()

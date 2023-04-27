clear; close all; clc;

% System parameters
m = 0.1;      % Mass of the pendulum [kg]
l = 0.5;      % Length of the pendulum [m]
M = 0.5;      % Mass of the reaction wheel [kg]
R = 0.1;      % Radius of the reaction wheel [m]
g = 9.81;     % Gravitational acceleration [m/s^2]
J = 1/3 * m * l^2 + 1/2 * M * R^2;  % Inertia of the system

N = 230;
u = [1 zeros(1,N-1)];

% State space representation of the system
A = [0, 1, 0, 0;
     3*m*g/(2*J), 0, 0, 1;
     0, 0, 0, 1;
     -(m*g*l)/(J), 0, 0, 0]

B = [0; -3/(2*J); 0; 1/J]

C = [1, 0, 0, 0;
     0, 0, 1, 0]

D = zeros(2, 1)

% Create state-space model
syms a3 a2 a1 s b0 b1

sys = ss(A, B, C, D);
Phi=inv(s*eye(4)- A)
H=C*Phi*B+D
pretty(H)

% Design a linear-quadratic regulator (LQR) controller
Q = diag([100, 1, 10, 1]);
R = 1;
[K, ~, ~] = lqr(sys, Q, R);

% Closed-loop system
sys_cl = ss(A - B * K, B, C, D);

% Initial conditions (pendulum starts at a 10-degree angle)
x0 = [deg2rad(100); 0; 0; 0];

% Simulate the system
t = linspace(0, 10, 1000);
[y, t] = initial(sys_cl, x0, t);

% Plot the results
figure;
subplot(2, 1, 1);
plot(t, rad2deg(y(:, 1)));
xlabel('Time [s]');
ylabel('Pendulum angle [deg]');

subplot(2, 1, 2);
plot(t, rad2deg(y(:, 2)));
xlabel('Time [s]');
ylabel('Reaction wheel angle [deg]');

sgtitle('Reaction Wheel Inverted Pendulum Simulation');

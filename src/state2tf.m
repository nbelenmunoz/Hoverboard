clear; close all; clc;
syms a3 a2 a1 s b0 b1

% System parameters
m = 0.1;      % Mass of the pendulum [kg]
l = 0.5;      % Length of the pendulum [m]
M = 0.5;      % Mass of the reaction wheel [kg]
R = 0.1;      % Radius of the reaction wheel [m]
g = 9.81;     % Gravitational acceleration [m/s^2]
J = 1/3 * m * l^2 + 1/2 * M * R^2;  % Inertia of the system

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
sys = ss(A, B, C, D);

%% Generate transfer function matrix with Matlab tf
% For each output
for i = 1:size(sys,1)

    % For each input
    for j = 1:size(sys,2)

        % Get SISO transfer function
        [num,den] = ss2tf(sys(i,j).A,sys(i,j).B,sys(i,j).C,sys(i,j).D);

        % Compute tf
        Gtf(i,j) = tf(num,den);
    end
end

%% Generate transfer function matrix symbolic
% Laplace variable
s = sym('s');

% For each output
for i = 1:size(sys,1)

    % For each input
    for j = 1:size(sys,2)

        % Get SISO transfer function
        [num,den] = ss2tf(sys(i,j).A,sys(i,j).B,sys(i,j).C,sys(i,j).D);

        % Compute tf
        Gsym(i,j) = poly2sym(num,s);
    end
end

% Den is the same for all elements
Gsym = Gsym / poly2sym(den,s);

% Numeric vectors is transformed to symbolic vector, to make everything
% readable change the variable precision arithmetic to 4 digits for output
pretty(vpa(Gsym,4))
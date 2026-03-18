clear all;
close all;
addpath(genpath('utils_link'));

%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
r_actual = 0.031;
d_actual = 0.164;
% wheels angles init
PHI_INIT = [0;0];
%% Eight-shape trajectory parameters
R = 0.4;
omega_trj = 2*pi;

%% EKF parameters
% EKF initil covariance
P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF process covariance
D = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF measurement noise (delta wheels angles)
R_2 = diag([ENCODER_QUANRIZATION/6,ENCODER_QUANRIZATION/6].^2);
% EKF measurement noise (GPS + delta wheels angles)
R_4 = diag(([0.001, 0.001, ENCODER_QUANRIZATION/6,ENCODER_QUANRIZATION/6]).^2);

%% Get an eight-shaped geometric path

% Sample the space variable s uniformly in [0,1]
N_samples = 1000; % arbitrary
s = linspace(0,1, N_samples); 

[x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot] = gen_eight_shape_trajectory(R, omega_trj, s);

%% Sample s and get q trajectory with differential flatness
% set Q_INIT to the initial q (will be used as initialization in simulink)
% set also Q_INIT_LOC (eventually with initial error)
% set also Z_INIT_EKF (eventually with initial error)

% (u contains geometric input)
[q, u] = cartisian_flatness(x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot);

Q_INIT = q(:,1);


%% Plot the trajectory

plot_unicycle_2D(q, 50);

%% Get timing law

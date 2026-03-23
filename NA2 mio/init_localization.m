clear all;
close all;
addpath(genpath('./'))
%%
T_s = 0.04; 
r = 0.03;
d = 0.165;
r_actual = 0.031;
d_actual = 0.164;
% wheels angles init
PHI_INIT = [0;0];

N_samples = 1000; % arbitrary
s = linspace(0,1, N_samples); 
R = 0.4;
omega_trj = 2*pi;
[x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot] = gen_eight_shape_trajectory(R, omega_trj, s);

[q, u] = cartisian_flatness(x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot);

Q_INIT = q(:,1);

plot_unicycle_2D(q,50);
clear all;
close all;
clc;

%%
T_SIMULATION = 10;
T_s = 0.04;
r_nominal = 0.03;
d_nominal = 0.165;
w_nominal = [r_nominal;r_nominal/d_nominal];

N_samples = 1000; % arbitrary
s = linspace(0,1, N_samples); 
R = 0.4;
omega_trj = 2*pi;
[x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot] = gen_eight_shape_trajectory(R, omega_trj, s);

[q, u] = cartisian_flatness(x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot);

Q_INIT = q(:,1);

PHI_INIT = [0;0];


%%

open_system('sim_identification.slx');
load_system('sim_identification.slx');
out = sim('sim_identification.slx',T_SIMULATION);

%%

q_sim = out.q_sim.signals.values;
omega_wheels_sim = out.omega_wheels_sim.signals.values;

N = size(q_sim, 3);

q4id = squeeze(q_sim)'; % -> (N x 3)
omega_wheels = squeeze(omega_wheels_sim)'; % -> (N x 2)
t = out.tout;

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));
%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
w_max = 10;

Ta = 1;
Tc = 30;

T_SIM = 2*Ta+Tc;
t = (0:T_s:T_SIM)';
N_samples = T_SIM * 1/T_s;

s = linspace(0,1, N_samples); 


%% Eight-shape trajectory parameters
R = 0.4;
omega_trj = 2*pi;
w = omega_trj;
[x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot] = gen_eight_shape_trajectory(R, omega_trj, s);

% MOTION PLANNING

clear all;
close all;
%addpath(genpath('utils'));
addpath(fullfile(pwd,'..','utils'));
  
%% Set simulation parameters
T_SIM = 1;
ki = 14;
kf = 10;

T_s = 0.001;

%% 2.1

qi = [-1; -1; pi/2];
qf = [1; 1; pi/2];

%% Time law
t = 0:T_s:T_SIM;
[s, s_dot] = time_law_constant(t, T_SIM);
[v,w] = trajectory_plan_cartisian(qi, qf, ki, kf, s,s_dot);
q = simulate_unicycle(qi, v, w, T_s);

plot_unicycle_2D(q,50);

%%



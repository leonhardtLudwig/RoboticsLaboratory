% MOTION PLANNING

clear all;
close all;
addpath(genpath('utils'));
  
%% Set simulation parameters
T_SIMULATION = 10;


%% 2.1

qi = [-1; -1; pi/2];
qf = [1; 1; pi/2];

%% Time law
t = 0:0.01:T_SIMULATION;
[s, s_dot] = time_law_trapezoidal(t);

%% Trajectory generation
ki = 5; 
kf = 5;

[v,w] = trajectory_plan_cartisian(qi, qf, ki, kf, s,s_dot);

%%

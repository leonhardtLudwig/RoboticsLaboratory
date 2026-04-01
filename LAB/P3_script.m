%% PARTE 3

clear all;
close all;
%addpath(genpath('utils'));
addpath(fullfile(pwd,'..','utils'));
  
%% Set simulation parameters

T_SIM = 20;  % tentativo

T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

%% Time Law params

%trapezoidal
Ta = 2;
Tc = 3;

%% Trajectory params
qi = [-0.5; -0.5; pi/2];
qf = [0.5; 0.5; pi/2];

ki = 5;
kf = 5;

%%
plot_unicycle_2D(q,50);

%%



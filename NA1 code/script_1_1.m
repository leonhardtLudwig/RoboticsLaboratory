clear all;
close all;
addpath(genpath('utils'));

%% Set simulation parameters
r = 0.03;
d = 0.165;
T_SIMULATION = 10;
Q_INITIAL = [0;0;0];

%% Set input
OMEGA_L = 10;
OMEGA_R = -10;

%% Velocità lineare e angolare

v = 5; 
w = 10;

%% Chained form

Z_INITIAL = [0;0;0];
v1_chained = 10;
v2_chained = 10;
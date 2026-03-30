%% Setup iniziale
clear all;
close all;
%addpath(genpath('utils'));
addpath(fullfile(pwd,'..','utils'));

%% Set simulation parameters
r = 0.03;
d = 0.165;
T_SIMULATION = 10;
Q_INITIAL = [0;0;0];

%% 1) Set input - Wheel velocities
OMEGA_L = 10; % Left wheel velocity
OMEGA_R = 10; % Right wheel velocity

%% 
OMEGA_L = 10; OMEGA_R = 5;
%%
OMEGA_L = 5;  OMEGA_R = 10;

%%
OMEGA_L = 10; OMEGA_R = -10;

%% Run simulation 1_1
modelName = 'sim_1_1';
simOut = sim(modelName);

% Extract state data
q_data = simOut.get('q_sim');
q_vector = q_data.signals.values;

% Plot results
figure('Name', 'Simulation Result');

plot_unicycle_2D(q_vector, 50);
title(['Robot Trajectory - ' modelName]);

%% Set input - Unicycle velocities 
v = 5; w = 0;

%% 
v = 0; w = 10;

%% 
v = 5; w = 10;

%% Run simulation 1_2
modelName = 'sim_1_2';
simOut = sim(modelName);

% Extract state data
q_data = simOut.get('q_sim');
q_plot = q_data.signals.values;

% Plot results
figure('Name', 'Simulation Result');

plot_unicycle_2D(q_plot, 50);
title(['Robot Trajectory - ' modelName]);

%% Chained form - OPTIONAL
% same input as 1_2

v = 0; w = 2.0;
T_SIMULATION = 1;
Q_INITIAL = [1;1;0];

u_vector = [v; w];

%% Run simulation
modelName = 'sim_1_2';
simOut = sim(modelName);

% Extract state data
t = q_data.time;

q_data = simOut.get('q_sim');
q_plot = q_data.signals.values;

z_data = simOut.get('z_sim');
z_plot = z_data.signals.values;

v_ch_data = simOut.get('v_chained_sim');
v_ch_plot = v_ch_data.signals.values;

% Plot results
figure('Name', 'Simulation Result');

plot_unicycle_2D(q_plot, 50);
title(['Robot Trajectory - ' modelName]);

%% Plot results
figure('Name', 'Simulation Result');

subplot_chained_form(q_plot, u_vector, z_plot, v_ch_plot, t);
%subplot_chained_formV2(q_plot, u_vector, z_plot, v_ch_plot, t);
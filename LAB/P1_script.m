clear all;
close all;
clc
addpath(fullfile(pwd,'..','utils'));
%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

%% Eight-shape trajectory parameters
R = 0.4;
omega_trj = 2*pi;

%% Get an eight-shaped geometric path

% Sample the space variable s uniformly in [0,1]
N_samples = 1000; % arbitrary
s = linspace(0,1, N_samples); 

[x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot] = gen_eight_shape_trajectory(R, omega_trj, s);

%% Sample s and get q trajectory with differential flatness

% (u contains geometric input)
[q, u] = cartisian_flatness(x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot);

Q_INIT = q(:,1);

%% Time law

Ta = 1;
Tc = 30;

%% EKF parameters

ENCODER_QUANTIZATION = 2 * pi / 4096;
    
% EKF initil covariance
P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF process covariance
D = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF measurement noise (delta wheels angles)
R_2 = diag([ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6].^2);
% EKF measurement noise (GPS + delta wheels angles)
R_4 = diag(([0.001, 0.001, ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6]).^2);

Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0]; 


%% RUN EXPERIMENT PART 1

%% Set Values

Ta = 1;
Tc_values = [30,18,45];

% Test Number i = 1,2,3
i = 1;
Tc = Tc_values(i);

%% Run Simulation (or manually run simulink)

simulink_model_name = 'Part1'; 
out = sim(simulink_model_name);

disp('Simulation completed');


%% 
results = struct('T_s', cell(1, length(Ts_values)), ...
              'Ta', [], ...
              'Tc', [], ...
              'q_desired', [], ...
              'q_loc_exact', [], ...
              'q_loc_kalman', [] );

[P_INIT_EKF, D, R_2, R_3, R_4] = initialize_kalman_cov(T_s);
        
% out = sim(simulink_model_name);
    
q_desired = out.q_des.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;
q_loc_kalman = out.z_EKF.signals.values;
    
plot_EKF_results(q_actual, q_loc_exact, z_estimate);
title(['EKF GPS: T_s = ', num2str(T_s), ' | p_{loss} = ', num2str(p_loss)]);
       
% save data
results_part1(i).T_s = T_s;
results_part1(i).Ta = Ta;
results_part1(i).q_desired = q_desired;
results_part1(i).q_loc_exact = q_loc_exact;
results_part1(i).q_loc_kalman = q_loc_kalman;
 

%% Plot
plot_EKF_results(q_desired, q_loc_exact, q_loc_kalman);

%%



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



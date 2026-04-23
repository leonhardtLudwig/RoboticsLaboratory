    %% Experimental Activity 1 (EA1): Planning, Localization, and Identification
%% PART 1

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));

%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

%% Eight-shape trajectory parameters 
R = 0.4;
omega_trj = 2*pi;

%% Get an eight-shaped geometric path (computed online, here we just need initial position)
% 
% % Sample the space variable s uniformly in [0,1]
% N_samples = 1000; % arbitrary
% s = linspace(0,1, N_samples); 
% 
% [x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot] = gen_eight_shape_trajectory(R, omega_trj, s);
% 
% %% Sample s and get q trajectory with differential flatness
% 
% % (u contains geometric input)
% [q, u] = cartisian_flatness(x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot);
% 
% Q_INIT = q(:,1);

Q_INIT = [0; 0; atan2((R*omega_trj), (2*R*omega_trj))];  % atan(2,1)
Q_INIT_LOC = Q_INIT;

%% Time law


%% EKF parameters

% ENCODER_QUANTIZATION = 2 * pi / 4096;
% 
% % EKF initil covariance
% P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% % EKF process covariance
% D = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% % EKF measurement noise (delta wheels angles)
% R_2 = diag([ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6].^2);
% % EKF measurement noise (GPS + delta wheels angles)
% R_4 = diag(([0.001, 0.001, ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6]).^2);

Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0]; 
PHI_INIT = [0;0];


%% RUN EXPERIMENT PART 1

%% Set Values

Ta = 1;
Tc_values = [30,18,45];

% Test Number i = 1,2,3
i = 3;
Tc = Tc_values(i);
T_SIM = Ta*2+Tc;

%% Run Simulation (or manually run simulink)

simulink_model_name = 'Part1_second_try'; 
out = sim(simulink_model_name,T_SIM);

disp('Simulation completed');


%% Save Data
%% Save Data
% Definizione del template della struct
results = struct('T_s', [], ...
                 'Ta', [], ...
                 'Tc', [], ...
                 'q_desired', [], ...
                 'gyro',[],...
                 'w_gyro',[],...
                 'acce', [],...
                 'q_motion_capture',[],...
                 'wheels_speed_desired',[],...
                 'wheels_speed_measured',[],...
                 'out_backup',[]);
      
out_backup = out; 
q_des = out.q_des.signals.values;
acce = out.acce.signals.values;
gyro = out.gyro.signals.values;
w_gyro = out.w_gyro.signals.values;
q_motion_capture = out.q_motion_capture.signals.values; 
ws_des = out.wheels_speed_des.signals.values;
ws_meas = out.wheels_speed_meas.signals.values;
      

results_part1(i).T_s = T_s;
results_part1(i).Ta = Ta;
results_part1(i).Tc = Tc;
results_part1(i).q_des = q_des;
results_part1(i).gyro = gyro;
results_part1(i).w_gyro = w_gyro;
results_part1(i).acce = acce;
results_part1(i).q_motion_capture = q_motion_capture;
results_part1(i).ws_des = ws_des;
results_part1(i).ws_meas = ws_meas;
results_part1(i).out_backup = out_backup;
      
%% Plot
q_motion_cap = results_part1(1).q_motion_capture;

plot_unicycle_2D(q_motion_cap',50);

%%
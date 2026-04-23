%% Experimental Activity 1 (EA1): Planning, Localization, and Identification
%% PART 3

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));
 
%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

r = 0.03316;
d = 0.18428;

r_id = 0.03316;
d_id = 0.18428;

%% 3.1
% Plan trajectory with trapezoidal time law

qi = [-0.5; -0.5; pi/2];
qf = [0.5; 0.5; pi/2];

ki = 4;
kf = 4;

Ta = 1;
Tc = 10;
T_SIM = 2*Ta + Tc;

% Time law
t = 0:T_s:T_SIM;
[s, s_dot] = time_law_trapezoidal(t, Ta, Tc);
    
% Traj generation
[v,w] = trajectory_plan_cartisian(qi, qf, ki, kf, s,s_dot);
    
%% 3.2 Test the trajectory 
q = simulate_unicycle(qi, v, w, T_s);
u = [v;w];
    
% Plot functions for 2D and time
plot_unicycle_2D(q,50)
plot_unicycle_wrt_time(q, u, T_s);

[q_dot, omega_wheels] = DDR_Unicycle(q, u, d, r);

plot_wheels_speed(omega_wheels, T_s);

fprintf('max wheel speed: %.3f rad/s\n', max(abs(omega_wheels(:))))

omega_wheels = omega_wheels';
N_samples = size(omega_wheels, 1);
t_array = (0:N_samples-1)' * T_s;

wheels_speed_des = omega_wheels(1:N_samples, :); 
wheels_speed_des_ts = timeseries(wheels_speed_des, t_array);
wheels_speed_des_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

% Initial State for localization

Q_INIT = q(:,1);
    
Q_INIT_LOC = Q_INIT;

Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0]; 
PHI_INIT = [0;0];

% Observation matrix H for EKF

% Encorder
H_enc = [0, 0, 0, 1, 0, 0, 0;
         0, 0, 0, 0, 1, 0, 0];

% IMU (wz_gyro)
H_gyro = [0, 0, 0, 0, 0, -r_id/d_id, r_id/d_id];
     
% Motion capture (State)
H_motion_cap = [1, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0]; 

% Covariance matrix

ENCODER_QUANTIZATION = 2 * pi / 4096;
var_IMU = 0.01;
var_motion_capture = 0.01;
 
% EKF initil covariance
P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF process covariance
D = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
    
% Encoder
R_1 = diag([ENCODER_QUANTIZATION/6, ENCODER_QUANTIZATION/6].^2);
   
% Encoder + IMU
R_2 = diag(([ENCODER_QUANTIZATION/6, ENCODER_QUANTIZATION/6, var_IMU]).^2);

% Encoder + IMU + motion capture
R_3 = diag(([var_motion_capture, var_motion_capture, var_motion_capture, ...
             ENCODER_QUANTIZATION/6, ENCODER_QUANTIZATION/6, var_IMU]).^2);


%% Set LOSS CASES
i = 3;

p_loss_values = [1.0, 0.99, 0];
p_loss = p_loss_values(i);     

%% Run Simulation

simulink_model_name = 'P3_SIM'; 

out = sim(simulink_model_name,T_SIM);

%% Save Data

out_backup = out;   
results_part3_second_try(i) = out_backup;


%%
% plot_unicycle_2D(results_part3(1).q_loc_kalman.signals.values, 50);
% 
% plot_unicycle_2D(results_part3(1).q_loc_exact.signals.values, 50);
% 
% plot_EKF_results(q_motion_capture, q_loc_exact, q_loc_EKF);


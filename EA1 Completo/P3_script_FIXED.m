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

%r = 0.03316;
%d = 0.18428;

r_id = 0.03316;
d_id = 0.18428;

%% 3.1
% Plan trajectory with trapezoidal time law



Ta = 1;
Tc = 10;
T_SIM = 2*Ta + Tc;

% Time law
t = 0:T_s:T_SIM;
[s, s_dot] = time_law_trapezoidal(t, Ta, Tc);
    
% S Traj generation
%r = 0.03316;
%d = 0.18428;
%qi = [-0.5; -0.5; pi/2];
%qf = [0.5; 0.5; pi/2];
%ki = 4;
%kf = 4;
%[v,w] = trajectory_plan_cartisian(qi, qf, ki, kf, s,s_dot);

% 8 shape generation
r = 0.03;
d = 0.165;
R_8_shape = 0.4;
omega_8_shape = 2*pi;
[v, w, wL_8, wR_8, q_des_8] = eight_shape_plan(s, s_dot, R_8_shape, omega_8_shape, r, d);
qi = q_des_8(:, 1);
    
%% 3.2 Test the trajectory 
q = simulate_unicycle(qi, v, w, T_s);
q_des = q;
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

%% 3.4

Q_INIT = q(:,1);
    
Q_INIT_LOC = Q_INIT;

[P_INIT_EKF, D, R_2, R_3, R_4] = initialize_kalman_cov(T_s);
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
var_motion_capture = 0.001;
 
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


%% VERSION WITH H FULL

% set simulation params
i = 3;

p_loss_values = [1.0, 0.99, 0];
p_loss = p_loss_values(i);     % useful just for case 3

%% SIMULAZIONE
simulink_model_name = 'Part3_simulazione'; 

out = sim(simulink_model_name,T_SIM);

%% Save Data

out_backup = out;   



%%
plot_unicycle_2D(results_part3(1).q_loc_kalman.signals.values, 50);

plot_unicycle_2D(results_part3(1).q_loc_exact.signals.values, 50);

plot_EKF_results(q_motion_capture, q_loc_exact, q_loc_EKF);

%%
plot(results_part3(1).wheels_speed_measured.signals.values(:,1))
%%
plot(results_part3(1).wheels_speed_measured.signals.values(:,2))
%%
plot(results_part1(1).wheels_speed_measured(:,1))
%%
plot(results_part1(1).wheels_speed_measured(:,2))

%% FOR SIMULATION PART
simulink_model_name = 'Part3_simulazione'; 
out = sim(simulink_model_name);

%%
w_gyro = out.w_gyro.signals.values;
%w_gyro = out.gyro.signals.values(3);
q_motion_capture = out.q_motion_capture.signals.values; 
%q_motion_capture = out.motion_capture.signals.values;

wheels_speed_desired = out.wheels_speed_des.signals.values;
wheels_speed_measured = out.wheels_speed_meas.signals.values;

q_loc_euler = out.q_loc_euler.signals.values;
%q_loc_rk2 = out.q_loc_rk.signals.values;
q_loc_rk2 = out.q_loc_rk2.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;

z_estimate = out.z_EKF.signals.values;
P_filt_EKF = out.P_filt_EKF.signals.values;

q_loc_EKF = z_estimate(1:3,:,:);

%%
plot_unicycle_2D(q_motion_capture,50);

plot_EKF_results(q_motion_capture, q_loc_exact, q_loc_EKF);
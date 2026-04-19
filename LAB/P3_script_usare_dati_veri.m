%% Experimental Activity 1 (EA1): Planning, Localization, and Identification
%% PART 3

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));
 
load('EA1_Part3_Data.mat');

results = results_part3(1);
disp(['Results with Ta, Tc: ', num2str(results.Ta), ', ', num2str(results.Tc)]);

T_SIM = 2*results.Ta + results.Tc;

%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

r = 0.03316;
d = 0.18428;

r_id = 0.03316;
d_id = 0.18428;

%% 
% tiro fuori i dati per comodità 
q_des = results.q_desired;
q_motion_capture = results.q_motion_capture.signals.values;
gyro = results.gyro.signals.values;
acce = results.acce.signals.values;
ws_des = results.wheels_speed_desired.signals.values;
ws_meas = results.wheels_speed_measured.signals.values;
q_loc_exact = results.q_loc_exact.signals.values;
q_loc_EKF = results.q_loc_kalman.signals.values;



%% ADATTO I DATI CORRETTAMENTE PER DARLI IN INGRESSO A PART3_CON_DATI_VERI

% wheel_speed_meas va invertito (enc prende R,L)
ws_meas_inv(:,1) = ws_meas(:,2);  
ws_meas_inv(:,2) = ws_meas(:,1);

ws_meas = ws_meas_inv;

% w_gyro va: estratta comp z, ribaltata, rad/s, bias 0.1 (calibrazione)
w_gyro_nominale = -gyro(:,3) * 0.1 * (pi / 180); 
% 2. Parametri di calibrazione estratti dall'analisi
k_gyro = 1.0444;
b_gyro = -0.005482; 
% 3. Calibrazione finale (Questa è la variabile da dare a Simulink!)
w_gyro = (w_gyro_nominale * k_gyro) + b_gyro;

% motion capture va tenuto così com'è (MODIFICARE IL SIMULINK TOGLIENDO I
% MENO)

%% Input per Simulink PART3_CON_DATI_VERI

%% Prepare Data for the Simulink
  
N_samples = size(ws_meas, 1);
t_array = (0:N_samples-1)' * T_s;

wheels_speed_meas_ts = timeseries(ws_meas, t_array);
wheels_speed_meas_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

ws_des = ws_des(1:N_samples, :); 
wheels_speed_des_ts = timeseries(ws_des, t_array);
wheels_speed_des_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

w_gyro = w_gyro(1:N_samples,:); 
w_gyro_ts = timeseries(w_gyro, t_array);
w_gyro_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

% calibration done online 
q_motion_capture = q_motion_capture(1:N_samples, :);
q_motion_capture_ts = timeseries(q_motion_capture, t_array);
q_motion_capture_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');


%% TEST LOCALIZZAZIONE CON DATI LAB

Q_INIT = q_motion_capture(1,:)';

% Q_INIT_LOC = Q_INIT
Q_INIT_LOC = Q_INIT + [0.2; 0.2; deg2rad(15)];

[P_INIT_EKF, D, R_2, R_3, R_4] = initialize_kalman_cov(T_s);
Z_INIT_EKF = [Q_INIT_LOC; 0; 0; 0; 0]; 
PHI_INIT = [0;0];

%% Observation matrix H for EKF

% Encorder
H_enc = [0, 0, 0, 1, 0, 0, 0;
         0, 0, 0, 0, 1, 0, 0];

% IMU (wz_gyro)
H_gyro = [0, 0, 0, 0, 0, -r_id/d_id, r_id/d_id];
     
% Motion capture (State)
H_motion_cap = [1, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0]; 

%% Covariance matrix

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


%% VERSION WITH H FULL

% set simulation params
i = 3;

p_loss_values = [1.0, 0.99, 0];
p_loss = p_loss_values(i);

%% RUN SIMULATION P3_CON_DATI_VERI

%% Collect Data

q_loc_euler = out.q_loc_euler.signals.values;
q_loc_rk2 = out.q_loc_rk2.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;

% plot_localization_results(q_motion_capture_cal', q_loc_euler, q_loc_rk2, q_loc_exact);

z_estimate = out.z_EKF.signals.values;
P_filt_EKF = out.P_filt_EKF.signals.values;

q_loc_EKF = z_estimate(1:3,:,:);


%%
plot_EKF_results(q_motion_capture', q_loc_exact, q_loc_EKF);


%% Numerical Activity 3 (NA3): Feedback control
%% PART 2: POSTURE REGULATION

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));

%% Set simulation parameters
r_nominal = 0.03;
d_nominal = 0.165;
%r = r_nominal;
%d = d_nominal;
r_actual = r_nominal;
d_actual = d_nominal;

omega_max = 10;
T_s = 0.04;


r_actual = 0.03316;
d_actual = 0.18428;

r = r_actual;
d = d_actual;




controller_index = 2; % 1->cartesian, 2->posture
flg_replanning = true;
% desired configuration
q_d = [0;0;0];
% initial configuration
Q_INIT = [0;-2;0];
% simulation time
T_SIM = 20;

%% Set controller parameters
if controller_index == 1
    % cartesian
    k_1 = 1; 
    k_2 = 10;
    control_par = [k_1, k_2, 0];
else
    % posture
    k_1 = 5; 
    k_2 = 5;
    k_3 = 10;
    control_par = [k_1, k_2, k_3];
end


%% 

%Q_INIT = q(:,1);
    
Q_INIT_LOC = Q_INIT;

Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0]; 
PHI_INIT = [0;0];

% Observation matrix H for EKF

% Encorder
H_enc = [0, 0, 0, 1, 0, 0, 0;
         0, 0, 0, 0, 1, 0, 0];

% IMU (wz_gyro)
H_gyro = [0, 0, 0, 0, 0, -r_actual/d_actual, r_actual/d_actual];
     
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
i = 1;

p_loss_values = [1.0, 0.99, 0];
p_loss = p_loss_values(i); 
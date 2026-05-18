%% TASK 1: POSTURE REGULATION

%clear all;
%close all;
addpath(genpath(fullfile(pwd,'..','utils')));
addpath('Functions');
%%

x1 = -2.9; y1 = -1.5;
x2 = -1.6; y2 = -1.1;
x3 = -1.5; y3 = -0.7;
x4 = -1.4; y4 = -0.3;
x5 = -0.6; y5 =  0.1;
x6 =  0.2; y6 =  1.5;
x7 =  1.0; y7 =  0.2;
x8 =  1.1; y8 =  1.5;
x9 =  2.9;



Q_INIT = [-2.2; 0.7; -pi/2];



%% 

T_SIM = 30;

r_nominal = 0.03;
d_nominal = 0.165;

omega_M = 8;
T_s = 0.04;

r_actual = 0.03293;
d_actual = 0.16040;


r = r_actual;
d = d_actual;

tol = 0.025;

q_d = [-1.6; -1.1; 0];


%k_1 = 1.5; 
%k_2 = 1.5;
%k_3 = 0.1;
%control_par = [1.5, 1.5, 0.1]; problemi angolo 
%control_par = [10, 10, 10]; buono ma satura

control_par = [1, 2, 3.5];



%% EKF

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
%P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
P_INIT_EKF = diag([0.1, 0.1, 0.2, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);


% EKF process covariance
    

sigma_motion_capture = 8e-3;    
sigma_enc = ENCODER_QUANTIZATION/sqrt(12); 
sigma_imu = 1e-2;        

sigma_dw = 0.0175/6; %delta wheels

D = diag([0.8e-3, 0.8e-3, 5e-3, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
%D = diag([0.1, 0.1, 0.2, sigma_dw, sigma_dw, sigma_dw*T_s,sigma_dw*T_s].^2);
    
% Encoder + IMU + motion capture
R_3 = diag(([sigma_motion_capture, sigma_motion_capture, sigma_motion_capture, ...
             sigma_enc, sigma_enc, sigma_imu]).^2);


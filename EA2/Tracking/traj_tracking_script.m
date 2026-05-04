%% Numerical Activity 3 (NA3): Feedback control
%% PART 1: TRAJECTORY TRACKING 

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
omega_M = 10;
T_s = 0.04;


r_actual = 0.03316;
d_actual = 0.18428;

r = r_actual;
d = d_actual;

controller_index = 3;   % 1->lin, 2->nonlin, 3->FL
trj_index = 2;          % 1->line, 2->circle, 3->square, 4->8-shaped
state_type = 5; % 1 = motion capture, 2 = motion capture CAL, 3 = loc euler, 4 = loc rk2, 5 = exact loc



% --- line (1/T_trj [m/s] along the Y axis)
if trj_index == 1
    Q_INIT = [1; 0.; pi/2];
    T_trj = 2;
    T_SIM = 10;
elseif trj_index == 2
    % --- circle (radius 0.5 [m] with angular vel 2*pi/T_trj)
    Q_INIT = [0.1; -0.2; pi/2];
    T_trj = 5;
    T_SIM = 10;
elseif trj_index == 3
    % --- square (side_length 1 [m] with linear velocity side_length/(T_trj/4)[m/s])
    Q_INIT = [0.1; 0.; pi/2];
    T_trj = 20;
    T_SIM = 40;
else 
    % --- 8-shape (R=0.4 [m] with period T_trj)
    Q_INIT = [.2; 0.; pi/2];
    T_trj = 18;
    T_SIM = T_trj*4;
end

%% Set controller parameters
if controller_index == 1
    % linear
    xi = 0.707; 
    a = 5;
    control_par = [xi, a, 0];
elseif controller_index ==2
    % nonlinear
    xi = 0.9; 
    b = 25;
    control_par = [xi, b, 0];
elseif controller_index ==3
    % feedback_linearization
    k1 = 10; 
    k2 = 10;
    b = 0.01;
    control_par = [k1, k2,b];
end

%%

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

sigma_motion_capture = 1e-3;        % fix
sigma_enc = ENCODER_QUANTIZATION/sqrt(12);  % theorical?
sigma_imu = 0.03;                   % from residual analysi

D = diag([1.5e-3, 1.5e-3, 1e-2, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
    
% Encoder + IMU + motion capture
R_3 = diag(([sigma_motion_capture, sigma_motion_capture, sigma_motion_capture, ...
             sigma_enc, sigma_enc, sigma_imu]).^2);


%% VERSION WITH H FULL

% set simulation params
i = 1;

p_loss_values = [1.0, 0.99, 0];
p_loss = p_loss_values(i); 



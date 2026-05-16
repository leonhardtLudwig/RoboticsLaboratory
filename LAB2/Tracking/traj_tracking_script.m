%% Numerical Activity 3 (NA3): Feedback control
%% PART 1: TRAJECTORY TRACKING 

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));
addpath(genpath(fullfile(pwd,'..','..','utils')));

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

r_actual = 0.03293;
d_actual = 0.16040;

r = r_actual;
d = d_actual;

%% check for max velocity (the same of EA1-part3)

Ta = 1;
Tc = 10;
T_SIM = 2*Ta + Tc;

% Time law
t = 0:T_s:T_SIM;
[s, s_dot] = time_law_trapezoidal(t, Ta, Tc);
    
% S Traj generation
qi = [-0.5; -0.5; pi/2];
qf = [0.5; 0.5; pi/2];
ki = 4;
kf = 4;
[v,w] = trajectory_plan_cartisian(qi, qf, ki, kf, s,s_dot);
q = simulate_unicycle(qi, v, w, T_s);
q_des = q;
u = [v;w];
    
% Plot functions for 2D and time
% plot_unicycle_2D(q,50)
% plot_unicycle_wrt_time(q, u, T_s);

[q_dot, omega_wheels] = DDR_Unicycle(q, u, d, r);

% plot_wheels_speed(omega_wheels, T_s);

fprintf('max wheel speed: %.3f rad/s\n', max(abs(omega_wheels(:))))

%% check for the circular traj.
T_trj = 12;     % 12 is the minumum value
w_circ = 2*pi/T_trj;
v_circ = w_circ * 0.5;

wL_max = (v_circ - (d_actual/2)*w_circ) / r_actual
wR_max = (v_circ + (d_actual/2)*w_circ) / r_actual


%% set simulation params
i = 2;

p_loss_values = [1.0, 0.99, 0.90, 0];
p_loss = p_loss_values(i); 

controller_index = 2;   % 1->lin, 2->nonlin, 3->FL
trj_index = 2;          % 2->circle, 5->S-traj


if trj_index == 2
    % --- circle (radius 0.5 [m] with angular vel 2*pi/T_trj)
    Q_INIT = [0.4; 0; pi/2];
    %Q_INIT = [0; 0; pi/2];
    T_trj = 15;
    T_SIM = 15;


elseif trj_index == 5

    % S Traj generation
    Q_INIT = [-0.2; -0.4; pi/2];
    Q_INIT = [-0.4; -0.5; pi/2];


    Ta = 1;     % hard coded in the simulink
    Tc = 10;
    T_SIM = 2*Ta + Tc;
    
end

Q_INIT_LOC = Q_INIT;

Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0]; 
PHI_INIT = [0;0];


%% Set controller parameters
if controller_index == 1
    % linear
    xi = 0.9; 
    a = 2;    % 1 to have zero saturation
    control_par = [xi, a, 0];
elseif controller_index ==2
    % nonlinear
    xi = 0.7;         % 0.7,40 validi
    b = 40;
    control_par = [xi, b, 0];
elseif controller_index ==3
    % feedback_linearization
    k1 = 2; 
    k2 = 2;
    b = 0.05;   % potrebbe dare problemi (divisione per zero)
    control_par = [k1, k2,b];
end

%% Localization params

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

sigma_motion_capture = 8e-3;        
sigma_enc = ENCODER_QUANTIZATION/sqrt(12);
sigma_imu = 1e-2;    
 
% EKF initil covariance
P_INIT_EKF = diag([0.1, 0.1, 0.1, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);

% EKF process covariance
D = diag([0.8e-3, 0.8e-3, 5e-3, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);    
% Encoder + IMU + motion capture
R_3 = diag(([sigma_motion_capture, sigma_motion_capture, sigma_motion_capture, ...
             sigma_enc, sigma_enc, sigma_imu]).^2);

%%





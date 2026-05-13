%% TASK 1: POSTURE REGULATION

%clear all;
%close all;
addpath(genpath(fullfile(pwd,'..','utils')));
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


%Q_INIT_1 =  [x1;y8;-pi/2];

Q_INIT_1 = [-2.2; 0.7; -pi/2];


%Q_INIT_2 =  [x1+(x3-x1)/2;y8;-pi/2];

%Q_INIT_3 =  [x3;y8;-pi/2];

%Q_INIT_4 =  [x2;y7;-pi/2];

Q_INIT_2 =  [x1;y7;-pi/2];

Q_INIT_3 =  [x3;y8;-pi/2];

Q_INIT_4 =  [x3;y7;-pi/2];



%% 

T_SIM = 30;

r_nominal = 0.03;
d_nominal = 0.165;

omega_M = 10;
T_s = 0.04;

%r_actual = 0.03293;
%d_actual = 0.16040;

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

control_par = [1.5, 1.5, 0.3];


Q_INIT = Q_INIT_1;

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

%% 

%q_WF = squeeze(out.q_WF.signals.values);

%plot_unicycle_trajectory(q_WF, q_WF, 'nada');

%%
Q_INIT = Q_INIT_1;
Q_INIT_LOC = Q_INIT;
Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0]; 
sim('TASK1_Regulation.slx')
q_WF1 = squeeze(ans.q_WF.signals.values);
ws_des_1 = ans.ws_des.signals.values;

%%
% Q_INIT = Q_INIT_2;
% Q_INIT_LOC = Q_INIT;
% Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0];
% sim('TASK1_Regulation.slx')
% q_WF2 = squeeze(ans.q_WF.signals.values);
% ws_des_2 = ans.ws_des.signals.values;
% 
% 
% %%
% Q_INIT = Q_INIT_3;
% Q_INIT_LOC = Q_INIT;
% Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0];
% sim('TASK1_Regulation.slx')
% q_WF3 = squeeze(ans.q_WF.signals.values);
% ws_des_3 = ans.ws_des.signals.values;
% 
% 
% 
% %%
% Q_INIT = Q_INIT_4;
% Q_INIT_LOC = Q_INIT;
% Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0];
% sim('TASK1_Regulation.slx')
% q_WF4 = squeeze(ans.q_WF.signals.values);
% ws_des_4 = ans.ws_des.signals.values;


% %%
% labels = {'Q1', 'Q2', 'Q3', 'Q4'};
% 
% 
% plot_4_unicycle_trajectories(q_WF1,q_WF2,q_WF3,q_WF4,labels,'Traj',1);
% plot_4_unicycle_orientation_error(q_WF1,q_WF2,q_WF3,q_WF4,labels,'Orientation Error',2);
% plot_4_wheel_velocities(ws_des_1,ws_des_2,ws_des_3,ws_des_4,labels,'Wheels speed',3);

%% 

load('dati_lab.mat')

q_ekf_lab = squeeze(out.q_EKF.signals.values);
q_mocap_cal_lab = squeeze(out.q_motion_capture_cal.signals.values);
q_model_lab = squeeze(out.q_model.signals.values);
vicon_gt = out.vicon_gt.signals.values';

ws_meas_lab = out.ws_meas.signals.values;
ws_des_lab = squeeze(out.ws_des.signals.values);
ws_nosat_lab = squeeze(out.wheels_speed_robot_input_without_sat.signals.values)';

labels = {'Q-SIM', 'Q-EKF-LAB', 'Q-MOCAP-CAL-LAB', 'Q-MODEL-LAB'};

plot_4_unicycle_trajectories(q_WF1,q_ekf_lab,vicon_gt,q_model_lab,labels,'Traj',4);
plot_4_unicycle_orientation_error(q_WF1,q_ekf_lab,q_mocap_cal_lab,q_model_lab,labels,'Ori error',5);

labels = {'WS-SIM', 'WS-MEAS-LAB', 'WS-DES-LAB', '--'};
plot_4_wheel_velocities(ws_des_1,ws_meas_lab,ws_des_lab,[0,0],labels,'ws',6);

%%
% z_EKF = squeeze(out.z_EKF.signals.values);
% P_EKF = out.P_filt_EKF.signals.values;
% 
% analyze_EKF_results(z_EKF,P_EKF,q_model_lab,5);
% plot_EKF_covariance_evolution(P_EKF,6);
 %% provo a calcolare displacement

 % delta_traj_mocap = q_model_lab-q_mocap_cal_lab;
 % delta_traj_ekf = q_model_lab-q_ekf_lab;
 % 
 % 
 % plot_unicycle_trajectory(delta_traj_mocap,delta_traj_ekf,'displacement');

 %% wheel speed input

% N = size(ws_des_lab, 2);
% t_vec = (0:N-1)' * 0.04;
% 
% input_wL = [t_vec, ws_des_lab(1, :)'];
% input_wR = [t_vec, ws_des_lab(2, :)'];

% N = size(ws_meas_lab, 1);
% t_vec = (0:N-1)' * 0.04;
% 
% input_wL = [t_vec, ws_meas_lab(:, 1)];
% input_wR = [t_vec, ws_meas_lab(:, 2)];

%%

%plot_unicycle_trajectory(q_model_lab,squeeze(out.q_ws_test.signals.values),'comp')

labels = {'Q-MODEL-LAB', 'Q-WS-TEST-MEAS', 'Q-EKF', 'Q-WF1'};

plot_4_unicycle_trajectories(q_model_lab,squeeze(out.q_ws_test.signals.values),q_ekf_lab,q_WF1,labels,'comp',10);
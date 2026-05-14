%% TASK 2

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

Q_INIT = [-1.6;-1.1;0];

%% 

T_SIM = 20;

Ta = 2;
Tc = T_SIM-2*Ta;

T_a_c = [Ta, Tc];

r_nominal = 0.03;
d_nominal = 0.165;

omega_M = 10;
T_s = 0.04;


r_actual = 0.03293;
d_actual = 0.16040;

r = r_actual;
d = d_actual;

%% Planning

qi = Q_INIT;
qf = [x7; y4; 0];

q = [qi, qf];

ki = 6; 
kf = 4;

k_trj = [ki,kf];

%% Controller 

controller_index = 1;   % 1->lin, 2->nonlin, 3->FL

if controller_index == 1
    % linear
    xi = 0.9; 
    a = 2;    % 1 to have zero saturation
    control_par = [xi, a, 0];
elseif controller_index ==2
    % nonlinear
    xi = 0.7; 
    b = 40;
    control_par = [xi, b, 0];
elseif controller_index ==3
    % feedback_linearization
    k1 = 2; 
    k2 = 2;
    b = 0.05;   % potrebbe dare problemi (divisione per zero)
    control_par = [k1, k2,b];
end

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
controller_index = 1;  

sim('TASK2.slx')
q_WF1 = squeeze(ans.q_WF.signals.values);
ws_des_1 = ans.ws_des.signals.values;

q_des = squeeze(ans.q_des.signals.values);

%%
controller_index = 2;  
sim('TASK2.slx')
q_WF2 = squeeze(ans.q_WF.signals.values);
ws_des_2 = ans.ws_des.signals.values;
%%
controller_index = 3;  
sim('TASK2.slx')
q_WF3 = squeeze(ans.q_WF.signals.values);
ws_des_3 = ans.ws_des.signals.values;

%%
labels = {'q_des', 'lin', 'nl', 'fl'};


plot_4_unicycle_trajectories(q_des,q_WF1,q_WF2,q_WF3,labels,'Traj',1);
plot_4_unicycle_orientation_error(q_des,q_WF1,q_WF2,q_WF3,labels,'Ori err',2);
plot_4_wheel_velocities(ws_des_1,ws_des_2,ws_des_3,[0,0],labels,'Wheels speed',3);

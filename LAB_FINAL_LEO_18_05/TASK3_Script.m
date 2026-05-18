%% TASK 2

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));
addpath(genpath(fullfile(pwd,'..','..','utils')));

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

Q_INIT = [x7; y4; 0];       % TASK 2 Final Position


% General Params
r_nominal = 0.03;
d_nominal = 0.165;
omega_M = 10;
T_s = 0.04;

r_actual = 0.03293;
d_actual = 0.16040;

r = r_actual;
d = d_actual;

%% Planning

T_line = 3;    % to go from x7 to x8
T_circ = 22;     

T_SIM = T_line + T_circ;

p_loss = 0.9;

%% Controller 

controller_index = 2;   % 1->lin, 2->nonlin, 3->FL

if controller_index == 1
    % linear
    xi = 0.9;           
    a = 2;    % 1 to have zero saturation
    control_par = [xi, a, 0];
elseif controller_index ==2
    % nonlinear
    xi =1;         % xi = 1 normal <1 rapid >1 slow
    b = 3;          % b alto per correggere errori laterali velocemente
    control_par = [xi, b, 0];   

    control_par1 = [1, 8, 0];   % corregge err inziale (sat) ma poi
    control_par2 = [0.4, 4, 0];   % non corregge l'errore inziale ma converge meglio alla fine
    control_par3 = [0.4, 12, 0];  % super performance
    control_par4 = [0.5, 8, 0];  % buon compromesso

    % TESTARE LA 3 e 4
    control_par = control_par3;
    
    %xi=1;   
    % b=5 inizia la sat (sicuro ma un pò lento)
    
    % abbassando xi inizio a convergere più velocemente ma errore inziale e
    % robot scattoso (NON SCENDERE SOTTO i 0.4)

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
D = diag([0.8e-3, 0.8e-3, 5e-3, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);

% Encoder + IMU + motion capture
sigma_motion_capture = 8e-3;    
sigma_enc = ENCODER_QUANTIZATION/sqrt(12); 
sigma_imu = 1e-2;

R_3 = diag(([sigma_motion_capture, sigma_motion_capture, sigma_motion_capture, ...
             sigma_enc, sigma_enc, sigma_imu]).^2);



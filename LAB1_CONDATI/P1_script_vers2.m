%% Experi+mental Activity 1 (EA1): Planning, Localization, and Identification
%% PART 1

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));

%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
w_max = 10;

%% Eight-shape trajectory parameters
R = 0.4;
omega_trj = 2*pi;

%% Get an eight-shaped geometric path

T_SIM = 2*Ta+Tc;
t = (0:T_s:T_SIM)';
N_samples = T_SIM * 1/T_s;

s = linspace(0,1, N_samples); 

[x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot] = gen_eight_shape_trajectory(R, omega_trj, s);

%% Sample s and get q trajectory with differential flatness

% (u contains geometric input)
[q, u] = cartisian_flatness(x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot);

Q_INIT = q(:,1);
Q_INIT_LOC = Q_INIT;


% Compute wheel velocities (Inverse Kinematics)
omega_wheels = []

% Verify hardware constraints
max_wL = max(abs(wL));
max_wR = max(abs(wR));

if max_wL > wM || max_wR > wM
    error('HARDWARE CONSTRAINT VIOLATED. Max speeds: wL=%.2f, wR=%.2f. Increase Tc.', max_wL, max_wR);
else
    fprintf('Hardware constraints respected. Max speeds: wL=%.2f, wR=%.2f\n', max_wL, max_wR);
end




%% Time law

Ta = 1;
Tc = 30;


%% EKF parameters

ENCODER_QUANTIZATION = 2 * pi / 4096;
    
% EKF initil covariance
P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF process covariance
D = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF measurement noise (delta wheels angles)
R_2 = diag([ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6].^2);
% EKF measurement noise (GPS + delta wheels angles)
R_4 = diag(([0.001, 0.001, ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6]).^2);

Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0]; 
PHI_INIT = [0;0];


%% RUN EXPERIMENT PART 1

%% Set Values

Ta = 1;
Tc_values = [30,18,45];

% Test Number i = 1,2,3
i = 3;
Tc = Tc_values(i);
T_SIM = Ta*2+Tc;


%% Set matrix H for EKF

%Encoder
H1 = [0, 0, 0, 1, 0, 0, 0;
      0, 0, 0, 0, 1, 0, 0];

%Angular velocities
H2 = [0, 0, 0, 0, 0, 1, 0;
      0, 0, 0, 0, 0, 0, 1]; 
%Configuration
H3 = [1, 0, 0, 0, 0, 0, 0;
      0, 1, 0, 0, 0, 0, 0;
      0, 0, 1, 0, 0, 0, 0]; 

H = H1;
%H = [H2;H1];

%% Run Simulation (or manually run simulink)

simulink_model_name = 'Part1'; 
out = sim(simulink_model_name,T_SIM);

disp('Simulation completed');


%% 
results = struct('T_s', [], ...
              'Ta', [], ...
              'Tc', [], ...
              'q_desired', [], ...
              'q_loc_exact', [], ...
              'q_loc_kalman', [], ...
              'acce', [],...
              'gyro',[],...
              'odometry',[],...
              'out_backup',[]);

[P_INIT_EKF, D, R_2, R_3, R_4] = initialize_kalman_cov(T_s);
        
% out = sim(simulink_model_name);
out_backup = out;
q_desired = out.q_des.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;
q_loc_kalman = out.z_EKF.signals.values;
odometry = out.odometry.signals.values;
acce = out.acce.signals.values;
gyro = out.gyro.signals.values;



plot_EKF_results(q_desired, q_loc_exact, q_loc_kalman);
       
% save data
results_part1(i).T_s = T_s;
results_part1(i).Ta = Ta;
results_part1(i).q_desired = q_desired;
results_part1(i).q_loc_exact = q_loc_exact;
results_part1(i).q_loc_kalman = q_loc_kalman;
results_part1(i).odometry = odometry;
results_part1(i).acce = acce;
results_part1(i).gyro = gyro;
results_part1(i).out_backup = out_backup;
 

%% Plot
plot_EKF_results(q_desired, q_loc_exact, q_loc_kalman);

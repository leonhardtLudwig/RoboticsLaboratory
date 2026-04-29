%% Experimental Activity 1 (EA1): Planning, Localization, and Identification
%% PART 2

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));

%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

%% Load Data from Part1 and extract configuration 1 (Tc = 30s)

load('Parte1/DATI_LABORATORIO_PARTE_1.mat');

results = results_part1(1); % extract T=30 datas

disp(['Results with Ta, Tc: ', num2str(results.Ta), ', ', num2str(results.Tc)]);

T_SIM = 2*results.Ta + results.Tc;

% extract data
q_des = results.q_des;
q_motion_capture = results.q_motion_capture;
w_gyro = results.w_gyro;
acce = results.acce;
ws_des = results.ws_des;
ws_meas = results.ws_meas;

%%
plot_unicycle_2D(q_motion_capture',50)
plot_wheels_speed(ws_meas',T_s)

%% 2.1 - IDENTIFICATION / CALIBRATION

% as in NA2
q4id = q_motion_capture;  
q4id(:,3) = unwrap(q_motion_capture(:,3));  % correct ±2π jump

omega_wheels = ws_meas;  
N_samples = size(q4id, 1) - 1;

[PHI, Y] = get_phi_reg(q4id, omega_wheels, T_s);
delta_X = Y(1:N_samples);
delta_Y = Y(N_samples+1:2*N_samples);
delta_theta = Y(2*N_samples+1:3*N_samples);
% compute unconstrained solution
w_unconstrained_hat = (PHI'*PHI)\PHI'*Y;
r_unconstrained_hat = w_unconstrained_hat(1);
d_unconstrained_hat = w_unconstrained_hat(1)/w_unconstrained_hat(2);
% compute constrained solution
w_constr_hat = lsqlin(PHI,Y,[],[],[],[],[0,0]);
r_constr_hat = w_constr_hat(1);
d_constr_hat = w_constr_hat(1)/w_constr_hat(2);
% compute estimate
Y_unconstrained_hat = PHI*w_unconstrained_hat;
delta_X_unconstrained_hat = Y_unconstrained_hat(1:N_samples);
delta_Y_unconstrained_hat = Y_unconstrained_hat(N_samples+1:2*N_samples);
delta_theta_unconstrained_hat = Y_unconstrained_hat(2*N_samples+1:3*N_samples);

%% Test identification with calibration
% set initial value
offset_0 =0;
% setup objective function (function of orientation offset)
f_SE = @(w) get_SE_id_and_calibration(q4id, omega_wheels, T_s, w);
% optimize the offset with nonlinear opt
offset_hat = fminsearch(f_SE,offset_0);
% compute [r, r/d, x_off*r/d, y_off*r/d] estimates
[PHI_cal, Y] = get_phi_reg_calibration(q4id, omega_wheels, T_s, offset_hat);


w_cal_hat = lsqlin(PHI_cal,Y,[],[],[],[],[0,0, -inf, -inf]);
r_cal_hat = w_cal_hat(1);
d_cal_hat = w_cal_hat(1)/w_cal_hat(2);
x_off_cal_hat = w_cal_hat(3)/w_cal_hat(2);
y_off_cal_hat = w_cal_hat(4)/w_cal_hat(2);
% compute estimates
Y_cal_hat = PHI_cal*w_cal_hat;
delta_X_cal_hat = Y_cal_hat(1:N_samples);
delta_Y_cal_hat = Y_cal_hat(N_samples+1:2*N_samples);
delta_theta_cal_hat = Y_cal_hat(2*N_samples+1:3*N_samples);



% new motion capture state calibrated
theta_cal = q4id(:, 3) - offset_hat;
x_cal = q4id(:, 1) - (x_off_cal_hat .* cos(theta_cal) - y_off_cal_hat .* sin(theta_cal));
y_cal = q4id(:, 2) - (x_off_cal_hat .* sin(theta_cal) + y_off_cal_hat .* cos(theta_cal));


q_motion_capture_cal = [x_cal, y_cal, theta_cal];

%% 
plot_unicycle_2D(q_motion_capture',50);
plot_unicycle_2D(q_motion_capture_cal',50);

%% Plot estimates
t = (0:N_samples-1) * T_s;  % vettore tempi
%t = results.out_backup.tout(0:N_samples-1);  % vettore tempi


figure()
subplot(3,1,1)
plot(t, delta_X, 'k', 'LineWidth', 2); hold on; grid on
plot(t, delta_X_unconstrained_hat(1:N_samples), 'r', 'LineWidth', 1.5)
plot(t, delta_X_cal_hat(1:N_samples), 'b', 'LineWidth', 1.5)
legend('\delta X', '\delta X ID', '\delta X ID + CAL')
ylabel('\delta X [m]')

subplot(3,1,2)
plot(t, delta_Y, 'k', 'LineWidth', 2); hold on; grid on
plot(t, delta_Y_unconstrained_hat, 'r', 'LineWidth', 1.5)
plot(t, delta_Y_cal_hat, 'b', 'LineWidth', 1.5)
legend('\delta Y', '\delta Y ID', '\delta Y ID + CAL')
ylabel('\delta Y [m]')

subplot(3,1,3)
plot(t, delta_theta, 'k', 'LineWidth', 2); hold on; grid on
plot(t, delta_theta_unconstrained_hat, 'r', 'LineWidth', 1.5)
plot(t, delta_theta_cal_hat, 'b', 'LineWidth', 1.5)
legend('\delta\theta', '\delta\theta ID', '\delta\theta ID + CAL')
ylabel('\delta\theta [rad]'); xlabel('time [s]')

%% Results summary
fprintf('--- IDENTIFICATION ---\n')
fprintf('r_unconstrained = %.5f m\n', r_unconstrained_hat)
fprintf('d_unconstrained = %.5f m\n', d_unconstrained_hat)
fprintf('r_constrained   = %.5f m\n', r_constr_hat)
fprintf('d_constrained   = %.5f m\n\n', d_constr_hat)

fprintf('--- CALIBRATION ---\n')
fprintf('r_cal     = %.5f m\n', r_cal_hat)
fprintf('d_cal     = %.5f m\n', d_cal_hat)
fprintf('x_off_cal = %.5f m\n', x_off_cal_hat)
fprintf('y_off_cal = %.5f m\n', y_off_cal_hat)
fprintf('theta_off = %.5f rad\n\n', offset_hat)

%% Error statistics (calibrated model)
E_X     = delta_X     - delta_X_cal_hat;
E_Y     = delta_Y     - delta_Y_cal_hat;
E_theta = delta_theta - delta_theta_cal_hat;

fprintf('--- ERROR STATS (ID + CAL) ---\n')
fprintf('X:     mean=%.2e  std=%.2e\n', mean(E_X), std(E_X))
fprintf('Y:     mean=%.2e  std=%.2e\n', mean(E_Y), std(E_Y))
fprintf('theta: mean=%.2e  std=%.2e\n', mean(E_theta), std(E_theta))




%% 2.2 - TEST LOCALIZATION STRATEGIES
r_actual = r_cal_hat;
d_actual = d_cal_hat;

% New Identified params: r_actual = 0.03293 ; d_actual = 0.16040
%% Prepare Data for the Simulink (Part2)

N_samples = size(ws_meas, 1);
t_array = (0:N_samples-1)' * T_s;

wheels_speed_meas_ts = timeseries(ws_meas, t_array);
wheels_speed_meas_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

wheels_speed_des = ws_des(1:N_samples, :); 
wheels_speed_des_ts = timeseries(wheels_speed_des, t_array);
wheels_speed_des_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

omega_gyro = w_gyro(1:N_samples, :); 
omega_gyro_ts = timeseries(omega_gyro, t_array);
omega_gyro_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

% from calibration
q_motion_capture_cal = q_motion_capture_cal(1:N_samples, :);
q_motion_capture_ts = timeseries(q_motion_capture_cal, t_array);
q_motion_capture_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

%% Initial State for localization

Q_INIT = q_motion_capture_cal(1,:)';

Q_INIT_LOC = Q_INIT;

Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0]; 
PHI_INIT = [0;0];


%% Observation matrix H for EKF

% Encorder
H_enc = [0, 0, 0, 1, 0, 0, 0;
         0, 0, 0, 0, 1, 0, 0];

% IMU (wz_gyro)
H_gyro = [0, 0, 0, 0, 0, -r_actual/d_actual, r_actual/d_actual];
     
% Motion capture (State)
H_motion_cap = [1, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0]; 

%% Covariance matrix
T_SIM = 15;
ENCODER_QUANTIZATION = 0.001; % 2 * pi / 4096
var_IMU = 0.1;
var_motion_capture = 0.001;
 
% EKF initil covariance
P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);

% EKF process covariance
D = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);

% [x,y,theta,deltaphiL,deltaphi_R,deltaphi_dotL,deltaphi_dotR]
% [position, .., heading (drift), enc_states, .. , .. , .. , ..]

D = diag([0.1, 0.1, 0.10, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
    
% Encoder + IMU + motion capture
R_3 = diag(([var_motion_capture, var_motion_capture, var_motion_capture, ...
             ENCODER_QUANTIZATION/6, ENCODER_QUANTIZATION/6, var_IMU]).^2);


%% VERSION WITH H FULL

% set simulation params

% test case 1: Only encoder 
% test case 2: Encoder + IMU 
% test case 3: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.9) 
% test case 4: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.99) 

test_case = 1;
p_loss_values = [1.0, 0.99, 0.9, 0.0];
p_loss = p_loss_values(3);     % useful just for case 3


%% Run simulation
simulink_model_name = 'Part2'; 
out = sim(simulink_model_name);

% Collect Data (optional)

q_loc_euler = out.q_loc_euler.signals.values;
q_loc_rk2 = out.q_loc_rk2.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;

% plot_localization_results(q_motion_capture_cal', q_loc_euler, q_loc_rk2, q_loc_exact);

z_estimate = out.z_EKF.signals.values;
P_filt_EKF = out.P_filt_EKF.signals.values;

q_loc_EKF = z_estimate(1:3,:,:);

plot_EKF_results(q_motion_capture', q_loc_exact, q_loc_EKF);


%% Save Data

% test case 1: Only encoder 
% test case 2: Encoder + IMU 
% test case 3: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.9) 
% test case 4: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.99) 
i = 2;

results_template = struct('T_s', [], ...
                          'Ta', [], ...
                          'Tc', [], ...
                          'p_loss', [], ...
                          'q_desired', [], ...
                          'q_loc_euler', [], ...
                          'q_loc_rk2', [],...
                          'q_loc_exact', [], ...
                          'motion_capture', [], ...
                          'q_motion_capture_cal', [], ...
                          'w_gyro', [], ...
                          'wheels_speed_des', [], ...
                          'wheels_speed_meas', [], ...
                          'P_filt_EKF', [], ...
                          'z_EKF', [], ...
                          'out_backup', []);


out_backup = out; 
q_loc_euler = out.q_loc_euler.signals.values;
q_loc_rk2 = out.q_loc_rk2.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;
z_estimate = out.z_EKF.signals.values;
P_filt_EKF = out.P_filt_EKF.signals.values;
      

results_part2(i).T_s = T_s;
results_part2(i).Ta = results.Ta; 
results_part2(i).Tc = results.Tc; 
results_part2(i).p_loss = p_loss;

results_part2(i).q_des = q_des;
results_part2(i).q_loc_euler = q_loc_euler;
results_part2(i).q_loc_rk2 = q_loc_rk2;
results_part2(i).q_loc_exact = q_loc_exact;
results_part2(i).q_motion_capture = q_motion_capture;
results_part2(i).q_motion_capture_cal = q_motion_capture_cal;
results_part2(i).w_gyro = w_gyro;
results_part2(i).ws_des = ws_des;
results_part2(i).ws_meas = ws_meas;
results_part2(i).P_filt_EKF = P_filt_EKF; 
results_part2(i).z_EKF = z_estimate;      

results_part2(i).out_backup = out;

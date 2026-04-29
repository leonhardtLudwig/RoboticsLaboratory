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

ENCODER_QUANTIZATION = 2 * pi / 4096;
var_IMU = 0.0001;
var_motion_capture = 0.001;

var_x_mocap  = 1.042408e-07;
var_y_mocap  = 2.377908e-08;
var_th_mocap = 1.944852e-07;
var_ws_L     = 1.960914e-07;
var_ws_R     = 1.960914e-07;
var_w_gyro   = 8.860510e-04;

sigma_motion_capture = 1e-3;        % fix
sigma_enc = ENCODER_QUANTIZATION/sqrt(12);  % theorical?
sigma_imu = 0.03;                   % from residual analysis
 
% EKF initil covariance
P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);

% EKF process covariance
D = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);

% [x,y,theta,deltaphiL,deltaphi_R,deltaphi_dotL,deltaphi_dotR]
% [position, .., heading (drift), enc_states, .. , .. , .. , ..]

D = diag([1.5e-3, 1.5e-3, 1e-2, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
    

% Encoder + IMU + motion capture
R_3 = diag(([sigma_motion_capture, sigma_motion_capture, sigma_motion_capture, ...
             sigma_enc, sigma_enc, sigma_imu]).^2);


%% VERSION WITH H FULL

% set simulation params

% test case 1: Only encoder 
% test case 2: Encoder + IMU 
% test case 3: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.9) 
% test case 4: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.99) 

test_case = 4;
p_loss_values = [1.0, 0.99, 0.9, 0.0];
p_loss = p_loss_values(1);     % useful just for case 3


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
%%

plot_ekf_error_analysis(t_array, q_loc_EKF, P_filt_EKF, q_motion_capture_cal)
% plot_ekf_uncertainty(t_array, q_loc_EKF, P_filt_EKF, q_motion_capture_cal)

%%
plot_EKF_results(q_motion_capture', q_loc_exact, q_loc_EKF);
plot_covariance_analysis(t_array, P_filt_EKF)


%% Save Data

% test case 1: Only encoder 
% test case 2: Encoder + IMU 
% test case 3: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.9) 
% test case 4: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.99) 
i = 4;

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


%%
estimate_dynamic_noise(ws_meas, w_gyro, q_motion_capture_cal, 2 * pi / 4096)

%%
compare_tc1_tc2_results(results_part2(1), results_part2(2));

%%
(results_part2(1).z_EKF - results_part2(2).z_EKF)



%%
function plot_ekf_error_analysis(time_vector, q_loc_EKF, P_filt_EKF, ground_truth)
    % PLOT_EKF_ERROR_ANALYSIS Plots the estimation error (EKF - Ground Truth) 
    % bounded by the 3-sigma confidence intervals.
    %
    % INPUTS:
    %   time_vector: array of time steps
    %   q_loc_EKF: [3 x 1 x N] array of EKF estimated states [x; y; theta]
    %   P_filt_EKF: [7 x 7 x N] array containing the P matrix at each step
    %   ground_truth: [N x 3] or [3 x N] array of ground truth states

    % Ensure time_vector is a column vector
    time_vector = time_vector(:);
    N = length(time_vector);

    % Automatically correct ground_truth orientation to be [N x 3]
    if size(ground_truth, 1) == 3 && size(ground_truth, 2) > 3
        ground_truth = ground_truth';
    end

    % Extract state estimates and force them to be column vectors
    est_x = squeeze(q_loc_EKF(1, 1, :)); est_x = est_x(:);
    est_y = squeeze(q_loc_EKF(2, 1, :)); est_y = est_y(:);
    est_theta = squeeze(q_loc_EKF(3, 1, :)); est_theta = est_theta(:);

    % Extract ground truth and force to column vectors
    gt_len = min(N, size(ground_truth, 1));
    gt_x = ground_truth(1:gt_len, 1); gt_x = gt_x(:);
    gt_y = ground_truth(1:gt_len, 2); gt_y = gt_y(:);
    gt_theta = ground_truth(1:gt_len, 3); gt_theta = gt_theta(:);

    % Pad ground truth with NaNs if it's shorter than the time vector
    if gt_len < N
        padding = NaN(N - gt_len, 1);
        gt_x = [gt_x; padding];
        gt_y = [gt_y; padding];
        gt_theta = [gt_theta; padding];
    end

    % Calculate the Estimation Error
    err_x = est_x - gt_x;
    err_y = est_y - gt_y;
    % For orientation, compute the shortest angular distance
    err_theta = angdiff(gt_theta, est_theta); 

    % Extract variances (diagonal elements of P) and force to column vectors
    var_x = squeeze(P_filt_EKF(1, 1, :)); var_x = var_x(:);
    var_y = squeeze(P_filt_EKF(2, 2, :)); var_y = var_y(:);
    var_theta = squeeze(P_filt_EKF(3, 3, :)); var_theta = var_theta(:);

    % Compute 3-sigma bounds
    sigma3_x = 3 * sqrt(var_x);
    sigma3_y = 3 * sqrt(var_y);
    sigma3_theta = 3 * sqrt(var_theta);

    % Ensure dimensions match before plotting
    if length(est_x) ~= N || length(var_x) ~= N
        error('Dimension mismatch: time_vector length (%d) does not match EKF data length (%d).', N, length(est_x));
    end

    % Plot Error for X Position
    figure('Name', 'EKF Error Analysis (EKF - Truth)', 'Color', 'w');
    
    subplot(3, 1, 1);
    hold on; grid on;
    % Create shaded region for uncertainty centered at zero
    fill([time_vector; flipud(time_vector)], ...
         [sigma3_x; flipud(-sigma3_x)], ...
         [0.8 0.8 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    plot(time_vector, err_x, 'b', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1); % Zero reference line
    ylabel('Error X [m]');
    title('Estimation Error bounded by \pm3\sigma Covariance');
    legend('\pm3\sigma Covariance', 'Estimation Error', 'Location', 'best');

    % Plot Error for Y Position
    subplot(3, 1, 2);
    hold on; grid on;
    fill([time_vector; flipud(time_vector)], ...
         [sigma3_y; flipud(-sigma3_y)], ...
         [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    plot(time_vector, err_y, 'r', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1);
    ylabel('Error Y [m]');

    % Plot Error for Theta
    subplot(3, 1, 3);
    hold on; grid on;
    fill([time_vector; flipud(time_vector)], ...
         [sigma3_theta; flipud(-sigma3_theta)], ...
         [0.8 1 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    plot(time_vector, err_theta, 'g', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1);
    ylabel('Error \theta [rad]');
    xlabel('Time [s]');
end

function plot_covariance_analysis(time_vector, P_filt_EKF)
    % PLOT_COVARIANCE_ANALYSIS Plots the standard deviation (1-sigma) of the 
    % primary states (x, y, theta) over time to evaluate EKF confidence.
    %
    % INPUTS:
    %   time_vector: array of time steps
    %   P_filt_EKF: [7 x 7 x N] array containing the P matrix at each step

    % Ensure time_vector is a column vector
    time_vector = time_vector(:);
    N = length(time_vector);

    % Extract standard deviations (sqrt of variance) for pose states
    % State 1 = x, State 2 = y, State 3 = theta
    sigma_x = sqrt(squeeze(P_filt_EKF(1, 1, :))); sigma_x = sigma_x(:);
    sigma_y = sqrt(squeeze(P_filt_EKF(2, 2, :))); sigma_y = sigma_y(:);
    sigma_theta = sqrt(squeeze(P_filt_EKF(3, 3, :))); sigma_theta = sigma_theta(:);

    % Ensure dimensions match before plotting
    if length(sigma_x) ~= N
        error('Dimension mismatch: time_vector length (%d) does not match P_filt_EKF length (%d).', N, length(sigma_x));
    end

    % Create figure
    figure('Name', 'EKF Covariance Analysis (1-Sigma)', 'Color', 'w');
    
    % Plot Standard Deviation for X
    subplot(3, 1, 1);
    plot(time_vector, sigma_x, 'b', 'LineWidth', 1.5);
    grid on;
    ylabel('\sigma_x [m]');
    title('Standard Deviation (\sigma) of Pose Estimates over Time');

    % Plot Standard Deviation for Y
    subplot(3, 1, 2);
    plot(time_vector, sigma_y, 'r', 'LineWidth', 1.5);
    grid on;
    ylabel('\sigma_y [m]');

    % Plot Standard Deviation for Theta
    subplot(3, 1, 3);
    plot(time_vector, sigma_theta, 'g', 'LineWidth', 1.5);
    grid on;
    ylabel('\sigma_\theta [rad]');
    xlabel('Time [s]');
end

function plot_ekf_uncertainty(time_vector, q_loc_EKF, P_filt_EKF, ground_truth)
    % PLOT_EKF_UNCERTAINTY Plots the estimated trajectory, the 3-sigma
    % confidence bounds, and overlays the ground truth trajectory.
    %
    % INPUTS:
    %   time_vector: array of time steps
    %   q_loc_EKF: [3 x 1 x N] array of EKF estimated states [x; y; theta]
    %   P_filt_EKF: [7 x 7 x N] array containing the P matrix at each step
    %   ground_truth: [N x 3] array of ground truth states [x, y, theta]

    % Ensure time_vector is a column vector
    time_vector = time_vector(:);
    N = length(time_vector);

    % Extract state estimates and rigorously force them to be column vectors
    est_x = squeeze(q_loc_EKF(1, 1, :)); est_x = est_x(:);
    est_y = squeeze(q_loc_EKF(2, 1, :)); est_y = est_y(:);
    est_theta = squeeze(q_loc_EKF(3, 1, :)); est_theta = est_theta(:);

    % Extract ground truth and force to column vectors
    % Adjust lengths safely in case of minor dimension mismatches at the end of the array
    gt_len = min(N, size(ground_truth, 1));
    gt_x = ground_truth(1:gt_len, 1); gt_x = gt_x(:);
    gt_y = ground_truth(1:gt_len, 2); gt_y = gt_y(:);
    gt_theta = ground_truth(1:gt_len, 3); gt_theta = gt_theta(:);

    % Pad ground truth with NaNs if it's shorter than the time vector
    % to avoid plotting errors
    if gt_len < N
        padding = NaN(N - gt_len, 1);
        gt_x = [gt_x; padding];
        gt_y = [gt_y; padding];
        gt_theta = [gt_theta; padding];
    end

    % Extract variances (diagonal elements of P) and force to column vectors
    var_x = squeeze(P_filt_EKF(1, 1, :)); var_x = var_x(:);
    var_y = squeeze(P_filt_EKF(2, 2, :)); var_y = var_y(:);
    var_theta = squeeze(P_filt_EKF(3, 3, :)); var_theta = var_theta(:);

    % Compute 3-sigma bounds (standard deviation = sqrt(variance))
    sigma3_x = 3 * sqrt(var_x);
    sigma3_y = 3 * sqrt(var_y);
    sigma3_theta = 3 * sqrt(var_theta);

    % Ensure dimensions match before plotting
    if length(est_x) ~= N || length(var_x) ~= N
        error('Dimension mismatch: time_vector length (%d) does not match EKF data length (%d).', N, length(est_x));
    end

    % Plot X Position
    figure('Name', 'EKF Uncertainty Bounds vs Ground Truth', 'Color', 'w');

    subplot(3, 1, 1);
    hold on; grid on;
    fill([time_vector; flipud(time_vector)], ...
         [est_x + sigma3_x; flipud(est_x - sigma3_x)], ...
         [0.8 0.8 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    plot(time_vector, est_x, 'b', 'LineWidth', 1.5);
    plot(time_vector, gt_x, 'k--', 'LineWidth', 1.5);
    ylabel('X Position [m]');
    title('EKF Estimates and \pm3\sigma Bounds vs Ground Truth');
    legend('Uncertainty (\pm3\sigma)', 'EKF Estimate', 'Ground Truth', 'Location', 'best');

    % Plot Y Position
    subplot(3, 1, 2);
    hold on; grid on;
    fill([time_vector; flipud(time_vector)], ...
         [est_y + sigma3_y; flipud(est_y - sigma3_y)], ...
         [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    plot(time_vector, est_y, 'r', 'LineWidth', 1.5);
    plot(time_vector, gt_y, 'k--', 'LineWidth', 1.5);
    ylabel('Y Position [m]');

    % Plot Theta
    subplot(3, 1, 3);
    hold on; grid on;
    fill([time_vector; flipud(time_vector)], ...
         [est_theta + sigma3_theta; flipud(est_theta - sigma3_theta)], ...
         [0.8 1 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    plot(time_vector, est_theta, 'g', 'LineWidth', 1.5);
    plot(time_vector, gt_theta, 'k--', 'LineWidth', 1.5);
    ylabel('\theta Orientation [rad]');
    xlabel('Time [s]');
end
%%

function R_matrix = estimate_dynamic_noise(ws_meas, w_gyro, q_motion_capture, encoder_quantization)
    % ESTIMATE_DYNAMIC_NOISE Extracts measurement noise covariance from
    % dynamic data using residual analysis (raw signal minus smoothed signal).

    % 1. Analytical Encoder Variance (Uniform Quantization)
    % The variance of a uniform distribution of width q is q^2 / 12
    var_enc = (encoder_quantization^2) / 12;
    var_ws_L = var_enc;
    var_ws_R = var_enc;

    % 2. Dynamic Noise Extraction via Residuals for IMU and Mocap
    % Use a zero-phase moving median to extract the underlying dynamic
    % trend without phase shift, then compute the variance of the residuals.
    window_size = 15; % Window length for smoothing, adjust if dynamics are too fast

    % IMU Residuals
    w_gyro_smooth = smoothdata(w_gyro, 'movmedian', window_size);
    var_w_gyro = var(w_gyro - w_gyro_smooth);

    % Motion Capture Residuals
    % Autocorrect motion capture dimensions to [N x 3]
    if size(q_motion_capture, 1) == 3 && size(q_motion_capture, 2) > 3
        q_motion_capture = q_motion_capture';
    end

    mocap_smooth = smoothdata(q_motion_capture, 'movmedian', window_size);
    residuals_mocap = q_motion_capture - mocap_smooth;

    var_x_mocap = var(residuals_mocap(:, 1));
    var_y_mocap = var(residuals_mocap(:, 2));

    % Handle angle wrap-around in theta residuals analytically
    residual_theta = atan2(sin(q_motion_capture(:, 3) - mocap_smooth(:, 3)), ...
                           cos(q_motion_capture(:, 3) - mocap_smooth(:, 3)));
    var_th_mocap = var(residual_theta);

    % 3. Construct the Measurement Noise Covariance Matrix (R)
    % Order assumes full observation: [x, y, theta, w_L, w_R, w_gyro]
    R_matrix = diag([var_x_mocap, var_y_mocap, var_th_mocap, var_ws_L, var_ws_R, var_w_gyro]);

    % Output the results to the Command Window
    fprintf('--- Dynamic Measurement Covariance Extraction (Residual Analysis) ---\n');
    fprintf('var_x_mocap  = %e [m^2]\n', var_x_mocap);
    fprintf('var_y_mocap  = %e [m^2]\n', var_y_mocap);
    fprintf('var_th_mocap = %e [rad^2]\n', var_th_mocap);
    fprintf('var_ws_L     = %e [(rad/s)^2]\n', var_ws_L);
    fprintf('var_ws_R     = %e [(rad/s)^2]\n', var_ws_R);
    fprintf('var_w_gyro   = %e [(rad/s)^2]\n', var_w_gyro);
end



%%
function compare_tc1_tc2_results(res_tc1, res_tc2)
    % COMPARE_TC1_TC2_RESULTS Generates a comparative analysis between two EKF 
    % scenarios focusing on estimation error and estimated uncertainty (1-sigma).
    %
    % INPUTS:
    %   res_tc1: struct containing results for Test Case 1 (Only Encoder)
    %   res_tc2: struct containing results for Test Case 2 (Encoder + IMU)

    % Process Test Case 1
    [t1, err1_x, err1_y, err1_th, sig1_x, sig1_y, sig1_th] = extract_metrics(res_tc1);
    
    % Process Test Case 2
    [t2, err2_x, err2_y, err2_th, sig2_x, sig2_y, sig2_th] = extract_metrics(res_tc2);

    % --- Plot 1: Estimation Error Comparison ---
    figure('Name', 'EKF Comparison: Estimation Error (TC1 vs TC2)', 'Color', 'w');
    
    subplot(3, 1, 1);
    hold on; grid on;
    plot(t1, err1_x, 'b', 'LineWidth', 1.5);
    plot(t2, err2_x, 'r', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1);
    ylabel('Error X [m]');
    title('Estimation Error: Only Encoder (Blue) vs Encoder + IMU (Red)');
    legend('TC1: Only Encoder', 'TC2: Encoder + IMU', 'Location', 'best');

    subplot(3, 1, 2);
    hold on; grid on;
    plot(t1, err1_y, 'b', 'LineWidth', 1.5);
    plot(t2, err2_y, 'r', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1);
    ylabel('Error Y [m]');

    subplot(3, 1, 3);
    hold on; grid on;
    plot(t1, err1_th, 'b', 'LineWidth', 1.5);
    plot(t2, err2_th, 'r', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1);
    ylabel('Error \theta [rad]');
    xlabel('Time [s]');

    % --- Plot 2: Uncertainty (1-Sigma) Comparison ---
    figure('Name', 'EKF Comparison: Estimated Uncertainty (TC1 vs TC2)', 'Color', 'w');
    
    subplot(3, 1, 1);
    hold on; grid on;
    plot(t1, sig1_x, 'b', 'LineWidth', 1.5);
    plot(t2, sig2_x, 'r', 'LineWidth', 1.5);
    ylabel('\sigma_x [m]');
    title('Estimated Standard Deviation: Only Encoder (Blue) vs Encoder + IMU (Red)');
    legend('TC1: Only Encoder', 'TC2: Encoder + IMU', 'Location', 'best');

    subplot(3, 1, 2);
    hold on; grid on;
    plot(t1, sig1_y, 'b', 'LineWidth', 1.5);
    plot(t2, sig2_y, 'r', 'LineWidth', 1.5);
    ylabel('\sigma_y [m]');

    subplot(3, 1, 3);
    hold on; grid on;
    plot(t1, sig1_th, 'b', 'LineWidth', 1.5);
    plot(t2, sig2_th, 'r', 'LineWidth', 1.5);
    ylabel('\sigma_\theta [rad]');
    xlabel('Time [s]');
end

function [t, err_x, err_y, err_th, sig_x, sig_y, sig_th] = extract_metrics(res)
    % EXTRACT_METRICS Helper function to align and extract errors and sigmas
    
    % Ensure correct orientation for ground truth [N x 3]
    gt = res.q_motion_capture;
    if size(gt, 1) == 3 && size(gt, 2) > 3
        gt = gt';
    end
    
    % Extract state estimates [N x 1]
    est_x = squeeze(res.z_EKF(1, 1, :)); est_x = est_x(:);
    est_y = squeeze(res.z_EKF(2, 1, :)); est_y = est_y(:);
    est_th = squeeze(res.z_EKF(3, 1, :)); est_th = est_th(:);
    
    % Reconstruct time vector
    N = length(est_x);
    t = (0:N-1)' * res.T_s;
    
    % Safely align ground truth length to estimation length
    gt_len = min(N, size(gt, 1));
    gt_x = gt(1:gt_len, 1);
    gt_y = gt(1:gt_len, 2);
    gt_th = gt(1:gt_len, 3);
    
    % Truncate estimates to match available ground truth
    est_x = est_x(1:gt_len);
    est_y = est_y(1:gt_len);
    est_th = est_th(1:gt_len);
    t = t(1:gt_len);
    
    % Compute estimation error
    err_x = est_x - gt_x;
    err_y = est_y - gt_y;
    % Analytical angular difference wrap-around
    err_th = atan2(sin(est_th - gt_th), cos(est_th - gt_th));
    
    % Extract 1-sigma uncertainties
    sig_x = sqrt(squeeze(res.P_filt_EKF(1, 1, 1:gt_len))); sig_x = sig_x(:);
    sig_y = sqrt(squeeze(res.P_filt_EKF(2, 2, 1:gt_len))); sig_y = sig_y(:);
    sig_th = sqrt(squeeze(res.P_filt_EKF(3, 3, 1:gt_len))); sig_th = sig_th(:);
end
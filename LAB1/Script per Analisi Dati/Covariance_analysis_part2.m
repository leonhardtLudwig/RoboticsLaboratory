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

% Load Data from Part2
load('results_part2.mat');

%%
% Scegli l'indice della prova che vuoi analizzare
idx = 3; 

% Collect Data
% Estraiamo direttamente dalla struct caricata
ws_meas              = results_part2(idx).ws_meas;
q_loc_euler          = results_part2(idx).q_loc_euler;
q_loc_rk2            = results_part2(idx).q_loc_rk2;
q_loc_exact          = results_part2(idx).q_loc_exact;
q_motion_capture     = results_part2(idx).q_motion_capture;
q_motion_capture_cal = results_part2(idx).q_motion_capture_cal;
z_estimate           = results_part2(idx).z_EKF;
P_filt_EKF           = results_part2(idx).P_filt_EKF;

q_loc_EKF = z_estimate(1:3,:,:);

N_samples = size(ws_meas, 1);
t_array = (0:N_samples-1)' * T_s;

%% Plot e Analisi
plot_localization_results(q_motion_capture_cal, q_loc_euler, q_loc_rk2, q_loc_exact);
plot_ekf_error_analysis(t_array, q_loc_EKF, P_filt_EKF, q_motion_capture_cal)
plot_ekf_uncertainty(t_array, q_loc_EKF, P_filt_EKF, q_motion_capture_cal)

%% Plot Risultati finali EKF
plot_EKF_results(q_motion_capture', q_loc_exact, q_loc_EKF);

plot_covariance_analysis(t_array, P_filt_EKF)




%%
estimate_dynamic_noise(ws_meas, w_gyro, q_motion_capture_cal, 2 * pi / 4096)

%%
compare_tc1_tc2_results(results_part2(1), results_part2(2));

%%
(results_part2(1).z_EKF - results_part2(2).z_EKF)



%%



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
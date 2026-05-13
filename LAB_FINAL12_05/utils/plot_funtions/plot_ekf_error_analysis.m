function plot_ekf_error_analysis(time_vector, q_loc_EKF, P_filt_EKF, ground_truth)
    % PLOT_EKF_ERROR_ANALYSIS Plots the estimation error (EKF - Ground Truth) 
    % bounded by the 3-sigma confidence intervals and prints average errors.
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

    % --- STATISTICAL ANALYSIS (Omitting NaNs) ---
    valid_idx = ~isnan(err_x); % Find indices where ground truth exists
    
    % Mean Error (Bias)
    mean_err_x = mean(err_x(valid_idx));
    mean_err_y = mean(err_y(valid_idx));
    mean_err_th = mean(err_theta(valid_idx));
    
    % Root Mean Square Error (RMSE)
    rmse_x = sqrt(mean(err_x(valid_idx).^2));
    rmse_y = sqrt(mean(err_y(valid_idx).^2));
    rmse_th = sqrt(mean(err_theta(valid_idx).^2));

    % fprintf('\n--- EKF Estimation Error Analysis ---\n');
    % fprintf('Mean Error (Bias):\n');
    % fprintf('  X     : %+.4f [m]\n', mean_err_x);
    % fprintf('  Y     : %+.4f [m]\n', mean_err_y);
    % fprintf('  Theta : %+.4f [rad]\n', mean_err_th);
    % fprintf('Root Mean Square Error (RMSE):\n');
    % fprintf('  X     : %.4f [m]\n', rmse_x);
    % fprintf('  Y     : %.4f [m]\n', rmse_y);
    % fprintf('  Theta : %.4f [rad]\n\n', rmse_th);
    % --------------------------------------------

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

    % --- PLOTTING ---
    figure('Name', 'EKF Error Analysis (EKF - Truth)', 'Color', 'w');
    
    % Plot Error for X Position
    subplot(3, 1, 1);
    hold on; grid on;
    % Create shaded region for uncertainty centered at zero
    fill([time_vector; flipud(time_vector)], ...
         [sigma3_x; flipud(-sigma3_x)], ...
         [0.8 0.8 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    plot(time_vector, err_x, 'b', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1); % Zero reference line
    ylabel('Error X [m]');
    title(sprintf('Estimation Error bounded by \\pm3\\sigma (RMSE X: %.3f m)', rmse_x));
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
    title(sprintf('RMSE Y: %.3f m', rmse_y));

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
    title(sprintf('RMSE \\theta: %.3f rad', rmse_th));
end
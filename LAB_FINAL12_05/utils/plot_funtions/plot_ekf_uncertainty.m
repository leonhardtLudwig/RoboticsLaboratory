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
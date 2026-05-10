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
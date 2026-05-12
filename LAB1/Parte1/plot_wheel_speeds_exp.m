function plot_wheel_speeds_exp(exp_data)
    % PLOT_WHEEL_SPEEDS_EXP Plots the desired and measured wheel speeds.
    %
    % Input:
    %   exp_data - Struct containing fields Tc, ws_des(Nx2) and ws_meas(Nx2)
    %% 1. Data extraction and time setup
    Tc = exp_data.Tc;
    ws_des = exp_data.ws_des;
    ws_meas = exp_data.ws_meas;
    
    T_s = 0.04; % Fixed sampling time
    N = size(ws_des, 1);
    t = (0:N-1) * T_s; % Time vector
    %% 2. Figure Setup
    titolo_fig = sprintf('Wheel Speeds Analysis (Tc = %g)', Tc);
    figure('Name', titolo_fig, 'Color', 'w', 'Position', [100, 100, 800, 600]);
    
    sgtitle(titolo_fig, 'FontWeight', 'bold', 'FontSize', 14);
    %% 3. Left Wheel Subplot (Column 1)
    subplot(2, 1, 1);
    hold on; grid on;
    plot(t, ws_meas(:, 1), 'b-', 'LineWidth', 1, 'DisplayName', 'Measured (Noisy)');
    plot(t, ws_des(:, 1), 'k--', 'LineWidth', 2, 'DisplayName', 'Desired (Simulation)');
    title('Left Wheel (\omega_L)');
    ylabel('Angular velocity [rad/s]');
    legend('Location', 'best');
    xlim([0, t(end)]);
    %% 4. Right Wheel Subplot (Column 2)
    subplot(2, 1, 2);
    hold on; grid on;
    plot(t, ws_meas(:, 2), 'r-', 'LineWidth', 1, 'DisplayName', 'Measured (Noisy)');
    plot(t, ws_des(:, 2), 'k--', 'LineWidth', 2, 'DisplayName', 'Desired (Simulation)');
    title('Right Wheel (\omega_R)');
    xlabel('Time [s]');
    ylabel('Angular velocity [rad/s]');
    legend('Location', 'best');
    xlim([0, t(end)]);
    
    hold off;
    
    %% 5. Noise Analysis (Terminal Output)
    % Calculate the error (residuals)
    err_L = ws_meas(:, 1) - ws_des(:, 1);
    err_R = ws_meas(:, 2) - ws_des(:, 2);
    
    % Standard deviation of the noise (Absolute noise level)
    std_L = std(err_L);
    std_R = std(err_R);
    
    % Peak absolute desired velocity (For relative noise calculation)
    peak_des_L = max(abs(ws_des(:, 1)));
    peak_des_R = max(abs(ws_des(:, 2)));
    
    % Min and Max desired velocity (Signal bounds)
    min_des_L = min(ws_des(:, 1));
    max_des_L = max(ws_des(:, 1));
    min_des_R = min(ws_des(:, 2));
    max_des_R = max(ws_des(:, 2));
    
    % Min and Max measured velocity (Actual bounds including noise)
    min_meas_L = min(ws_meas(:, 1));
    max_meas_L = max(ws_meas(:, 1));
    min_meas_R = min(ws_meas(:, 2));
    max_meas_R = max(ws_meas(:, 2));
    
    % Relative Noise Percentage (Noise vs Signal Peak)
    if peak_des_L > 0 && peak_des_R > 0
        rel_noise_L = (std_L / peak_des_L) * 100;
        rel_noise_R = (std_R / peak_des_R) * 100;
    else
        rel_noise_L = 0; rel_noise_R = 0;
    end
    
    % --- Print Human-Readable Output ---
    fprintf('\n--- NOISE ANALYSIS (Tc = %g) ---\n', Tc);
    fprintf('Left Wheel (\\omega_L):\n');
    fprintf('  Noise Std Dev    : %.4f rad/s\n', std_L);
    fprintf('  Min Desired Vel  : %.4f rad/s\n', min_des_L);
    fprintf('  Max Desired Vel  : %.4f rad/s\n', max_des_L);
    fprintf('  Min Measured Vel : %.4f rad/s\n', min_meas_L);
    fprintf('  Max Measured Vel : %.4f rad/s\n', max_meas_L);
    fprintf('  Relative Noise   : %.2f %%\n', rel_noise_L);
    
    fprintf('Right Wheel (\\omega_R):\n');
    fprintf('  Noise Std Dev    : %.4f rad/s\n', std_R);
    fprintf('  Min Desired Vel  : %.4f rad/s\n', min_des_R);
    fprintf('  Max Desired Vel  : %.4f rad/s\n', max_des_R);
    fprintf('  Min Measured Vel : %.4f rad/s\n', min_meas_R);
    fprintf('  Max Measured Vel : %.4f rad/s\n', max_meas_R);
    fprintf('  Relative Noise   : %.2f %%\n', rel_noise_R);
    fprintf('-----------------------------------\n\n');
end
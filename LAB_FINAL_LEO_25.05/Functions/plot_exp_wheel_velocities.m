function plot_exp_wheel_velocities(exp, plot_title, fig_num)
    % PLOT_WHEEL_VELOCITIES Confronta velocità misurate vs desiderate delle due ruote.
    
    % Setup della figura
    fig = figure(fig_num); 
    clf(fig);
    set(fig, 'Name', 'Wheel Velocities Comparison', 'Color', 'w');
    sgtitle(sprintf('Wheel Velocities - %s', plot_title), 'FontWeight', 'bold', 'FontSize', 12, 'Interpreter', 'none');
    
    % --- ASSE DEI TEMPI ---
    Ts = 0.04; % [s]
    N = size(exp.ws_meas, 1);
    t = (0:N-1) * Ts; 
    
    % ==========================================
    % SUBPLOT 1: Velocità Ruota 1 (Colonna 1)
    % ==========================================
    subplot(2, 1, 1);
    hold on; grid on;
    
    % Desired vs Measured per Ruota 1
    plot(t, exp.ws_des(:, 1), '--', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.5, 'DisplayName', 'Desired (\omega_R)');
    plot(t, exp.ws_meas(:, 1), '-', 'Color', [0 0.4 0.8], 'LineWidth', 1.5, 'DisplayName', 'Measured (\omega_R)');
    
    ylabel('Velocity [rad/s]'); 
    title('Motor 1 (e.g., Right Wheel)');
    xlim([0, t(end)]);
    legend('Location', 'best');
    
    % ==========================================
    % SUBPLOT 2: Velocità Ruota 2 (Colonna 2)
    % ==========================================
    subplot(2, 1, 2);
    hold on; grid on;
    
    % Desired vs Measured per Ruota 2
    plot(t, exp.ws_des(:, 2), '--', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.5, 'DisplayName', 'Desired (\omega_L)');
    plot(t, exp.ws_meas(:, 2), '-', 'Color', [0 0.4 0.8], 'LineWidth', 1.5, 'DisplayName', 'Measured (\omega_L)');
    
    xlabel('Time [s]');
    ylabel('Velocity [rad/s]'); 
    title('Motor 2 (e.g., Left Wheel)');
    xlim([0, t(end)]);
    % (Legenda omessa qui per evitare ridondanza, ma attivabile se necessario)
    
end
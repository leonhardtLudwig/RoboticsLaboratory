function plot_polar_states(exp, exp_title, fig_num)
    % PLOT_POLAR_STATES Genera i grafici delle variabili di stato in 
    % coordinate polari (rho, gamma, delta, theta') per task di regulation.
    
    fig = figure(fig_num);
    clf(fig);
    set(fig, 'Name', 'Polar State Variables', 'Color', 'w');
    sgtitle(sprintf('Polar Coordinates Evolution - %s', exp_title), 'Interpreter', 'none', 'FontWeight', 'bold');
    
    Ts = 0.04;
    N = size(exp.polar_states, 2);
    t = (0:N-1) * Ts;
    
    % Estrazione delle variabili dalla struct
    rho         = exp.polar_states(1, :);
    gamma       = exp.polar_states(2, :);
    delta       = exp.polar_states(3, :);
    theta_prime = exp.polar_states(4, :);
    
    colors = lines(4);
    
    % --- 1. RHO (Distanza) ---
    subplot(2, 2, 1); hold on; grid on;
    plot(t, rho, 'Color', colors(1,:), 'LineWidth', 2, 'DisplayName', '\rho (Distance)');
    plot([0 t(end)], [0 0], 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    ylabel('\rho [m]', 'FontWeight', 'bold');
    title('Distance to Target (\rho)');
    xlim([0, t(end)]); legend('Location', 'best');
    
    % --- 2. GAMMA ---
    subplot(2, 2, 2); hold on; grid on;
    plot(t, gamma, 'Color', colors(2,:), 'LineWidth', 2, 'DisplayName', '\gamma');
    plot([0 t(end)], [0 0], 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    ylabel('\gamma [rad]', 'FontWeight', 'bold');
    title('Sagittal Angle Error (\gamma)');
    xlim([0, t(end)]); legend('Location', 'best');
    
    % --- 3. DELTA ---
    subplot(2, 2, 3); hold on; grid on;
    plot(t, delta, 'Color', colors(3,:), 'LineWidth', 2, 'DisplayName', '\delta');
    plot([0 t(end)], [0 0], 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    xlabel('Time [s]'); ylabel('\delta [rad]', 'FontWeight', 'bold');
    title('Heading to Target Angle (\delta)');
    xlim([0, t(end)]); legend('Location', 'best');
    
    % --- 4. THETA PRIME (Errore Orientamento) ---
    subplot(2, 2, 4); hold on; grid on;
    plot(t, theta_prime, 'Color', colors(4,:), 'LineWidth', 2, 'DisplayName', '\theta''');
    plot([0 t(end)], [0 0], 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    xlabel('Time [s]'); ylabel('\theta'' [rad]', 'FontWeight', 'bold');
    title('Orientation Error (\theta'')');
    xlim([0, t(end)]); legend('Location', 'best');
end
function plot_control_signals_regulation(exp, exp_title, fig_num)
    % PLOT_CONTROL_SIGNALS Plotta i comandi di velocità lineare e angolare
    % generati dal controllore (prima della conversione in wheel speeds).
    
    % Seleziona o crea la figura e la pulisce
    fig = figure(fig_num);
    clf(fig);
    set(fig, 'Name', 'Control Signals (v, \omega)', 'Color', 'w');
    
    sgtitle(sprintf('Control Signals - %s', exp_title), 'Interpreter', 'none', 'FontWeight', 'bold');
    
    % Asse dei tempi
    Ts = 0.04; 
    N = size(exp.u, 1);
    t = (0:N-1) * Ts;
    
    % ==========================================
    % SUBPLOT 1: Velocità Lineare (v)
    % ==========================================
    subplot(2, 1, 1);
    hold on; grid on;
    
    % Assumendo che 'v' sia la prima colonna
    plot(t, exp.u(:, 1), 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'Command v');
    
    ylabel('Velocity v [m/s]');
    title('Linear Velocity Command');
    xlim([0, t(end)]);
    legend('Location', 'best');
    
    % ==========================================
    % SUBPLOT 2: Velocità Angolare (\omega)
    % ==========================================
    subplot(2, 1, 2);
    hold on; grid on;
    
    % Assumendo che '\omega' sia la seconda colonna
    plot(t, exp.u(:, 2), 'LineWidth', 1.5, 'Color', [0.9290 0.6940 0.1250], 'DisplayName', 'Command \omega');
    
    xlabel('Time [s]');
    ylabel('Velocity \omega [rad/s]');
    title('Angular Velocity Command');
    xlim([0, t(end)]);
    legend('Location', 'best');
end
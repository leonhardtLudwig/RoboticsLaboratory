function plot_ws_actual_vs_sat(w_req, w_sat, T_s)
    
    % Resolves dimension mismatch between unscaled/scaled arrays silently
    N = min(size(w_req, 2), size(w_sat, 2));
    w_req = w_req(:, 1:N);
    w_sat = w_sat(:, 1:N);
    t_local = (0 : N-1) * T_s;
    
    wL_req = w_req(1,:);
    wR_req = w_req(2,:);
    wL_sat = w_sat(1,:);
    wR_sat = w_sat(2,:);
    
    figure('Name', 'Wheels Speed: Requested vs Saturated', 'Color', 'w');
    
    % WL Subplot
    subplot(2, 1, 1);
    plot(t_local, wL_req, 'b--', 'LineWidth', 1.5); hold on;
    plot(t_local, wL_sat, 'r', 'LineWidth', 1.5);
    ylabel('\omega_L [rad/s]', 'FontWeight', 'bold');
    legend('Requested', 'Saturated', 'Location', 'best');
    xlim([0, t_local(end)]);
    grid on;
    
    % WR Subplot
    subplot(2, 1, 2);
    plot(t_local, wR_req, 'b--', 'LineWidth', 1.5); hold on;
    plot(t_local, wR_sat, 'r', 'LineWidth', 1.5);
    ylabel('\omega_R [rad/s]', 'FontWeight', 'bold');
    xlabel('Time [s]', 'FontWeight', 'bold');
    legend('Requested', 'Saturated', 'Location', 'best');
    xlim([0, t_local(end)]);
    grid on;
end
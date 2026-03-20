function plot_localization_results(q_actual, q_euler, q_rk2, q_exact)

%   PLOT_LOCALIZATION_RESULTS Compares the estimated trajectories with the actual one.
    
    % squeeze -> 3xN
    q_actual = squeeze(q_actual)';
    q_euler  = squeeze(q_euler)';
    q_rk2    = squeeze(q_rk2)';
    q_exact  = squeeze(q_exact)';
    

    %% Figure 1: 2D Geometric Path (X vs Y)
    figure('Name', 'Localization Paths (X-Y)', 'Color', 'w');
    hold on; grid on; axis equal;
    
    % Plot trajectories with different styles for distinguishability
    plot(q_actual(:,1), q_actual(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'Actual Path');
    plot(q_euler(:,1), q_euler(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Euler');
    plot(q_rk2(:,1), q_rk2(:,2), 'b-.', 'LineWidth', 1.5, 'DisplayName', 'RK2');
    plot(q_exact(:,1), q_exact(:,2), 'g:', 'LineWidth', 2, 'DisplayName', 'Exact');
    
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Path Comparison: Actual vs Estimated');
    legend('Location', 'best');
    
%% Figure 2: State-Error in time ?? 
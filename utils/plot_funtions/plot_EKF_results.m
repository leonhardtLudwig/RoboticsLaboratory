function plot_EKF_results(q_actual, q_exact, z_estimate)

%   PLOT_LOCALIZATION_RESULTS Compares the estimated trajectories with the actual one.
    
    % squeeze
    q_actual = squeeze(q_actual)';
    q_exact = squeeze(q_exact)';
    z_estimate = squeeze(z_estimate)';
      
    %% Figure 1: 2D Geometric Path (X vs Y)
    figure('Name', 'Kalman Filter Estimate Paths (X-Y)', 'Color', 'w');
    hold on; grid on; axis equal;
    
    % Plot trajectories with different styles for distinguishability
    plot(q_actual(:,1), q_actual(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'Actual Path');
    plot(q_exact(:,1), q_exact(:,2), 'g:', 'LineWidth', 2, 'DisplayName', 'Localization Exact');
    plot(z_estimate(:,1), z_estimate(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Kalman Estimate');
    
    xlabel('x');
    ylabel('y');
    title('Path Comparison: Actual vs Loc Exact vs Kalman Estimate');
    legend('Location', 'best');

end
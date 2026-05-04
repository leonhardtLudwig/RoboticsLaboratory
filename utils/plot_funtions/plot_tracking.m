function plot_tracking(q_desired, q_actual)

    q_desired = squeeze(q_desired)';
    q_actual = squeeze(q_actual)';

    N = min(size(q_actual, 2), size(q_desired, 2));
    q_actual = q_actual(:, 1:N);
    q_desired = q_desired(:, 1:N);
    
    figure('Name', 'Cartesian Trajectory Tracking', 'Color', 'w');
    hold on; grid on; axis equal;

    plot(q_desired(:,1), q_desired(:,2), 'b--', 'LineWidth', 1.5); hold on;
    plot(q_actual(:,1), q_actual(:,2), 'r', 'LineWidth', 1.5);
    
    xlabel('X [m]', 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontWeight', 'bold');
    legend('Desired Path', 'Actual Path', 'Start', 'End', 'Location', 'best');
    axis equal;
    grid on;
end
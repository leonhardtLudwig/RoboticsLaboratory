function plot_unicycle_wrt_time(q, u, T_s)
    
    % q has a value at the end we don't need
    if size(q, 2) > size(u, 2)
        q = q(:, 1:end-1);
    end

    N = size(q, 2);
    t_local = (0 : N-1) * T_s;

    figure('Name', 'Trajectory wrt Time', 'Color', 'w');
    % X
    subplot(5, 1, 1);
    plot(t_local, q(1,:), 'b', 'LineWidth', 1.5);
    ylabel('x [m]', 'FontWeight', 'bold');
    xlim([0, t_local(end)]);
    grid on;

    % Y
    subplot(5, 1, 2);
    plot(t_local, q(2,:), 'b', 'LineWidth', 1.5);
    ylabel('y [m]', 'FontWeight', 'bold');
    xlim([0, t_local(end)]);
    grid on;

    % Theta
    subplot(5, 1, 3);
    plot(t_local, q(3,:), 'b', 'LineWidth', 1.5);
    ylabel('\theta [rad]', 'FontWeight', 'bold');
    xlim([0, t_local(end)]);
    grid on;

    % V
    subplot(5, 1, 4);
    plot(t_local, u(1,:), 'b', 'LineWidth', 1.5);
    ylabel('v [m/s]', 'FontWeight', 'bold');
    xlim([0, t_local(end)]);
    grid on;

    % W
    subplot(5, 1, 5);
    plot(t_local, u(2,:), 'b', 'LineWidth', 1.5);
    ylabel('w [rad/s]', 'FontWeight', 'bold');
    xlabel('Time [s]', 'FontWeight', 'bold');
    xlim([0, t_local(end)]);
    grid on;

end
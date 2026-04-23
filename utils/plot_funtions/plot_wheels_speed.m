function plot_wheels_speed(omega_wheels, T_s)
    
    wL = omega_wheels(1,:);
    wR = omega_wheels(2,:);

    N = size(wL, 2);
    t_local = (0 : N-1) * T_s;

    figure('Name', 'Wheels speed wrt Time', 'Color', 'w');
    % WL
    subplot(2, 1, 1);
    plot(t_local, wL, 'r', 'LineWidth', 1.5);
    ylabel('\omega_L [rad/s]', 'FontWeight', 'bold');
    xlim([0, t_local(end)]);
    grid on;
    
    % WR
    subplot(2, 1, 2);
    plot(t_local, wR, 'r', 'LineWidth', 1.5);
    ylabel('\omega_R [rad/s]', 'FontWeight', 'bold');
    xlabel('Time [s]', 'FontWeight', 'bold');
    xlim([0, t_local(end)]);
    grid on;

end
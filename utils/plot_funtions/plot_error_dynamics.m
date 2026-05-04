function plot_error_dynamics(err, T_s)
    % Extracts arrays and safely generates local time vector
    err = squeeze(err);
    N = size(err);
    t_local = (0 : N-1) * T_s;
    
    ex = err(1,:);
    ey = err(2,:);
    etheta = err(3,:);
    
    % Vectorized norm of the Cartesian error
    norm_e = sqrt(ex.^2 + ey.^2);
    
    figure('Name', 'Error Dynamics', 'Color', 'w');
    plot(t_local, norm_e, 'k', 'LineWidth', 1.5);
    ylabel('norme_e [m]', 'FontWeight', 'bold');
    xlabel('Time [s]', 'FontWeight', 'bold');
    xlim([0, t_local(end)]);
    grid on;


    % figure('Name', 'Error Dynamics', 'Color', 'w');
    % % e_x
    % subplot(4, 1, 1);
    % plot(t_local, ex, 'b', 'LineWidth', 1.5);
    % ylabel('e_x [m]', 'FontWeight', 'bold');
    % xlim([0, t_local(end)]);
    % grid on;
    % 
    % % e_y
    % subplot(4, 1, 2);
    % plot(t_local, ey, 'b', 'LineWidth', 1.5);
    % ylabel('e_y [m]', 'FontWeight', 'bold');
    % xlim([0, t_local(end)]);
    % grid on;
    % 
    % % e_theta
    % subplot(4, 1, 3);
    % plot(t_local, etheta, 'b', 'LineWidth', 1.5);
    % ylabel('e_\theta [rad]', 'FontWeight', 'bold');
    % xlim([0, t_local(end)]);
    % grid on;
    % 
    % % Error Norm
    % subplot(4, 1, 4);
    % plot(t_local, norm_e, 'k', 'LineWidth', 1.5);
    % ylabel('||e_{xy}|| [m]', 'FontWeight', 'bold');
    % xlabel('Time [s]', 'FontWeight', 'bold');
    % xlim([0, t_local(end)]);
    % grid on;
end
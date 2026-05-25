function compare_multiple_regulation_experiments(exps, exp_labels, start_fig)
    % COMPARE_MULTIPLE_REGULATION_EXPERIMENTS Confronta N esperimenti su 5 plot separati.
    % Include traiettorie, velocità, errori cartesiani, segnali di controllo
    % e variabili di stato in coordinate polari (rho, gamma, delta, theta').
    
    num_exps = length(exps);
    if num_exps ~= length(exp_labels)
        error('Il numero di esperimenti e di etichette deve coincidere.');
    end
    
    % Genera una palette di colori dinamica
    colors = lines(num_exps); 
    Ts = 0.04; % Tempo di campionamento
    
    % =========================================================================
    % PLOT 1: TRAIETTORIE q_EKF (con simboli Unicycle a Triangolo)
    % =========================================================================
    fig1 = figure(start_fig); clf(fig1);
    set(fig1, 'Name', 'EKF Trajectories Comparison', 'Color', 'w');
    hold on;
    
    % --- 1.1 Definizione e Disegno Ambiente ---
    x1 = -2.90; x3 = -1.50; x4 = -1.40; x5 = -0.60; x6 = 0.20; 
    x7 = 1.00; x9 = 1.80; x10 = 2.20; x11 = 2.90;
    y1 = -1.50; y3 = -0.70; y5 = -0.20; y6 = 0.10; y7 = 0.20; y8 = 1.50;
    
    patch([x1 x3 x3 x1], [y6 y6 y8 y8], [0.75 0.85 0.95], 'EdgeColor', 'none', 'FaceAlpha', 0.6, 'HandleVisibility', 'off');
    text(x1+0.2, (y6+y8)/2, 'INITIAL', 'FontWeight', 'bold', 'FontSize', 8);
    patch([x5 x6 x6 x5], [y1 y1 y6 y6], [1.0 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
    patch([x7 x11 x11 x7], [y1 y1 y8 y8], [0.9 0.6 0.4], 'EdgeColor', 'none', 'FaceAlpha', 0.4, 'HandleVisibility', 'off');
    patch([x4 x5 x5 x4], [y3 y3 y6 y6], [0.5 0.5 0.5], 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'HandleVisibility', 'off');
    patch([x6 x7 x7 x6], [y1 y1 y3 y3], [0.5 0.5 0.5], 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'HandleVisibility', 'off');
    patch([x9 x10 x10 x9], [y5 y5 y7 y7], [0.5 0.3 0.2], 'FaceAlpha', 0.9, 'EdgeColor', 'k', 'HandleVisibility', 'off');
    plot([x4 x6], [y1 y1], 'k', 'LineWidth', 2.5, 'HandleVisibility', 'off');
    plot([x7 x5], [y6 y6], 'k', 'LineWidth', 2.5, 'HandleVisibility', 'off');
    
    % --- 1.2 Parametri Geometrici Robot (per simboli a triangolo) ---
    all_x = [x1 x11]; all_y = [y1 y8];
    span = max(max(all_x)-min(all_x), max(all_y)-min(all_y));
    base_L = span * 0.035; 
    base_vertices = [base_L, 0; -0.3*base_L, 0.4*base_L; -0.3*base_L, -0.4*base_L];
    num_symbols_to_plot = 8;
    
    % Target fisso di regulation
    plot(-1.6, -1.1, 'kx', 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', 'Target');
    
    % --- 1.3 Disegno Traiettorie e Simboli ---
    for i = 1:num_exps
        q = exps{i}.q_EKF;
        current_color = colors(i,:);
        
        % A. Plotta la linea continua
        plot(q(1,:), q(2,:), 'Color', current_color, 'LineWidth', 2, 'DisplayName', exp_labels{i});
        
        % B. Plotta i simboli a triangolo
        plot_trajectory_symbols_comparison(q(1,:), q(2,:), q(3,:), ...
                                        base_vertices, num_symbols_to_plot, ...
                                        current_color, 0.4, 0.5);
    end
    
    axis equal; grid on;
    xlim([x1-0.2, x11+0.2]); ylim([y1-0.2, y8+0.2]);
    xlabel('X [m]', 'FontWeight', 'bold'); ylabel('Y [m]', 'FontWeight', 'bold');
    title('EKF Trajectories Comparison (Regulation)', 'FontSize', 14);
    legend('Location', 'northeastoutside');
    hold off;
    
    % =========================================================================
    % PLOT 2: VELOCITÀ MISURATE (ws_meas)
    % =========================================================================
    fig2 = figure(start_fig + 1); clf(fig2);
    set(fig2, 'Name', 'Wheel Velocities Comparison', 'Color', 'w');
    sgtitle('Measured Wheel Velocities (\omega_R, \omega_L)', 'FontWeight', 'bold');
    
    subplot(2, 1, 1); hold on; grid on;
    for i = 1:num_exps
        N = size(exps{i}.ws_meas, 1);
        t = (0:N-1) * Ts;
        plot(t, exps{i}.ws_meas(:, 1), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', exp_labels{i});
    end
    ylabel('Velocity [rad/s]'); title('Motor 1 (Right)');
    legend('Location', 'best');
    
    subplot(2, 1, 2); hold on; grid on;
    for i = 1:num_exps
        N = size(exps{i}.ws_meas, 1);
        t = (0:N-1) * Ts;
        plot(t, exps{i}.ws_meas(:, 2), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', exp_labels{i});
    end
    xlabel('Time [s]'); ylabel('Velocity [rad/s]'); title('Motor 2 (Left)');
    hold off;
    
    % =========================================================================
    % PLOT 3: REGULATION ERROR (EKF vs DESIRED)
    % =========================================================================
    fig3 = figure(start_fig + 2); clf(fig3);
    set(fig3, 'Name', 'Regulation Errors Comparison', 'Color', 'w');
    sgtitle('Regulation Errors (EKF vs Desired)', 'FontWeight', 'bold');
    
    err_labels = {'Error X [m]', 'Error Y [m]', 'Error \theta [rad]'};
    
    for state_idx = 1:3
        subplot(3, 1, state_idx); hold on; grid on;
        for i = 1:num_exps
            N = size(exps{i}.err_ekf_des, 2);
            t = (0:N-1) * Ts;
            plot(t, exps{i}.err_ekf_des(state_idx, :), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', exp_labels{i});
        end
        ylabel(err_labels{state_idx});
        
        if state_idx == 1
            legend('Location', 'best');
        end
        if state_idx == 3
            xlabel('Time [s]');
        end
    end
    hold off;
    
    % =========================================================================
    % PLOT 4: SEGNALI DI CONTROLLO (u = [v, omega])
    % =========================================================================
    fig4 = figure(start_fig + 3); clf(fig4);
    set(fig4, 'Name', 'Control Signals Comparison', 'Color', 'w');
    sgtitle('Control Signals (v, \omega)', 'FontWeight', 'bold');
    
    subplot(2, 1, 1); hold on; grid on;
    for i = 1:num_exps
        N = size(exps{i}.u, 1);
        t = (0:N-1) * Ts;
        plot(t, exps{i}.u(:, 1), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', exp_labels{i});
    end
    ylabel('Velocity v [m/s]'); title('Linear Velocity (v)');
    legend('Location', 'best');
    
    subplot(2, 1, 2); hold on; grid on;
    for i = 1:num_exps
        N = size(exps{i}.u, 1);
        t = (0:N-1) * Ts;
        plot(t, exps{i}.u(:, 2), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', exp_labels{i});
    end
    xlabel('Time [s]'); ylabel('Velocity \omega [rad/s]'); title('Angular Velocity (\omega)');
    hold off;
    
    % =========================================================================
    % PLOT 5: VARIABILI DI STATO POLARI (rho, gamma, delta, theta')
    % =========================================================================
    fig5 = figure(start_fig + 4); clf(fig5);
    set(fig5, 'Name', 'Polar Coordinates Comparison', 'Color', 'w');
    sgtitle('Polar Coordinates Evolution Comparison', 'FontWeight', 'bold');
    
    var_names = {'\rho [m]', '\gamma [rad]', '\delta [rad]', '\theta'' [rad]'};
    var_titles = {'Distance to Target (\rho)', 'Sagittal Angle Error (\gamma)', ...
                  'Heading to Target Angle (\delta)', 'Orientation Error (\theta'')'};
    
    for var_idx = 1:4
        subplot(2, 2, var_idx); hold on; grid on;
        
        for i = 1:num_exps
            if isfield(exps{i}, 'polar_states')
                N = size(exps{i}.polar_states, 2);
                t = (0:N-1) * Ts;
                plot(t, exps{i}.polar_states(var_idx, :), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', exp_labels{i});
            else
                warning('L''esperimento "%s" non contiene il campo polar_states.', exp_labels{i});
            end
        end
        
        % Disegna linea orizzontale a zero per riferimento
        if var_idx > 1
            plot([0, t(end)], [0, 0], 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
        end
        
        ylabel(var_names{var_idx}, 'FontWeight', 'bold');
        title(var_titles{var_idx});
        
        if var_idx == 1
            legend('Location', 'best');
        end
        if var_idx > 2
            xlabel('Time [s]');
        end
    end
    hold off;
end

% =========================================================================
% FUNZIONE HELPER LOCALE per i simboli a triangolo
% =========================================================================
function plot_trajectory_symbols_comparison(x, y, theta, base_v, num_target, color, alpha, border_w)
    N = length(x);
    if N < 2, return; end
    step = max(1, round(N / num_target));
    indices = unique([1, step:step:N, N]);
    
    for i = indices
        xi = x(i); yi = y(i); th_i = theta(i);
        
        % Matrice di rotazione (R2D)
        R = [cos(th_i), -sin(th_i); 
             sin(th_i),  cos(th_i)];
         
        % Applica rotazione e traslazione
        rotated_v = (R * base_v')';
        translated_v = rotated_v + [xi, yi];
        
        % Disegna il poligono (patch)
        patch('Vertices', translated_v, 'Faces', [1 2 3], ...
              'FaceColor', color, 'FaceAlpha', alpha, ...
              'EdgeColor', color, 'LineWidth', border_w, ...
              'HandleVisibility', 'off'); 
    end
end
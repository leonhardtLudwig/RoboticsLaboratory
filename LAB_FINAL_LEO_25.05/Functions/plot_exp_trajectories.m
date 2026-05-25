function plot_exp_trajectories(exp, plot_title, fig_num, task_type)
    % PLOT_EXP_TRAJECTORIES Plotta le traiettorie dell'unicycle nell'ambiente
    %
    % Input:
    %   exp        - Struct dell'esperimento
    %   plot_title - Titolo del grafico
    %   fig_num    - Numero della figura
    %   task_type  - Stringa: 'regulation' oppure 'tracking'
    
    % --- 0. PREPARAZIONE DATI DALLA STRUCT ---
    % Prepariamo un cell array dinamico con le traiettorie disponibili
    trajs = {exp.q_model, exp.gt};
    legend_labels = {'Model (q\_model)', 'Ground Truth (gt)'};
    
    % Colori RGB
    colors = {[0 0 1], [0 0.7 0]}; % Blu per Model, Verde per GT
    
    % Aggiungiamo EKF se esiste nella struct
    if isfield(exp, 'q_EKF')
        trajs{end+1} = exp.q_EKF;
        legend_labels{end+1} = 'EKF (q\_EKF)';
        colors{end+1} = [1 0 0]; % Rosso
    end
    
    % --- 1. DEFINIZIONE VARIABILI AMBIENTE ---
    x1 = -2.90; x2 = -1.60; x3 = -1.50; x4 = -1.40; x5 = -0.60;
    x6 = 0.20;  x7 = 1.00;  x8 = 1.45; x9 = 1.80; x10 = 2.20; x11 = 2.90;
    y1 = -1.50; y2 = -1.10; y3 = -0.70; y4 = -0.30; y5 = -0.20;
    y6 = 0.10;  y7 = 0.20;  y8 = 1.50;
    
    % Setup figura
    fig = figure(fig_num); 
    clf(fig);
    set(fig, 'Name', 'Environment Analysis', 'Color', 'w');
    hold on;
    
    % --- 2. DISEGNA LE ZONE COLORATE ---
    % Zona Blu (senza testo)
    patch([x1 x3 x3 x1], [y6 y6 y8 y8], [0.75 0.85 0.95], 'EdgeColor', 'none', 'FaceAlpha', 0.6, 'DisplayName', 'Initial Position');
    
    % Zona Rossa (Corridoio)
    patch([x5 x6 x6 x5], [y1 y1 y6 y6], [1.0 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
    % Zona Arancione (Target Area)
    patch([x7 x11 x11 x7], [y1 y1 y8 y8], [0.9 0.6 0.4], 'EdgeColor', 'none', 'FaceAlpha', 0.4, 'HandleVisibility', 'off');
    
    % --- 3. DISEGNA GLI OSTACOLI ---
    obs_gray = [0.5 0.5 0.5];
    obs_brown = [0.5 0.3 0.2];
    patch([x4 x5 x5 x4], [y3 y3 y6 y6], obs_gray, 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Ostacoli');
    patch([x6 x7 x7 x6], [y1 y1 y3 y3], obs_gray, 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    patch([x9 x10 x10 x9], [y5 y5 y7 y7], obs_brown, 'FaceAlpha', 0.9, 'EdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % --- 4. DISEGNA LE LINEE NERE (Confini) ---
    plot([x4 x6], [y1 y1], 'k', 'LineWidth', 2.5, 'HandleVisibility', 'off');
    plot([x7 x5], [y6 y6], 'k', 'LineWidth', 2.5, 'HandleVisibility', 'off');
    
    % --- 5. LOGICA TARGET/TRAIETTORIA DESIDERATA ---
    if strcmpi(task_type, 'regulation')
        % Plotta il target fisso con una croce nera
        plot(-1.6, -1.1, 'kx', 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', 'Target Position');
    elseif strcmpi(task_type, 'tracking')
        % Plotta la traiettoria desiderata
        if isfield(exp, 'q_des')
            plot(exp.q_des(1, :), exp.q_des(2, :), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Desired Trajectory');
        else
            warning('Attenzione: task_type è Tracking ma exp.q_des non è presente nella struct.');
        end
    end
    
    % --- 6. DISEGNA TRAIETTORIE E SIMBOLI ---
    for i = 1:length(trajs)
        plot(trajs{i}(1, :), trajs{i}(2, :), 'Color', colors{i}, 'LineWidth', 1.8, 'DisplayName', legend_labels{i});
    end
    
    % Calcolo span per dimensioni dinamiche del robot
    all_x = [x1 x11]; all_y = [y1 y8];
    span = max(max(all_x)-min(all_x), max(all_y)-min(all_y));
    base_L = span * 0.035; 
    base_vertices = [base_L, 0; -0.3*base_L, 0.4*base_L; -0.3*base_L, -0.4*base_L];
    num_symbols_target = 6;
    
    for i = 1:length(trajs)
        plot_trajectory_symbols(trajs{i}(1,:), trajs{i}(2,:), trajs{i}(3,:), ...
                                base_vertices, num_symbols_target, colors{i}, 0.5, 0.5);
    end
    
    % Grafica finale
    axis equal; grid on;
    xlim([x1-0.2, x11+0.2]); ylim([y1-0.2, y8+0.2]);
    xlabel('X [m]', 'FontWeight', 'bold'); ylabel('Y [m]', 'FontWeight', 'bold');
    title(plot_title, 'FontSize', 14, 'Interpreter', 'none');
    legend('Location', 'northeastoutside');
    hold off;
end

% =========================================================================
% FUNZIONE HELPER LOCALE per disegnare il triangolo del robot
% =========================================================================
function plot_trajectory_symbols(x, y, theta, base_v, num_target, color, alpha, border_w)
    N = length(x);
    if N < 2, return; end
    step = max(1, round(N / num_target));
    indices = unique([1, step:step:N, N]);
    for i = indices
        xi = x(i); yi = y(i); th_i = theta(i);
        
        % Matrice di rotazione
        R = [cos(th_i), -sin(th_i); sin(th_i), cos(th_i)];
        
        % Ruota e trasla i vertici
        rotated_v = (R * base_v')';
        translated_v = rotated_v + [xi, yi];
        
        % Plotta il triangolo
        patch('Vertices', translated_v, 'Faces', [1 2 3], 'FaceColor', color, ...
              'FaceAlpha', alpha, 'EdgeColor', color, 'LineWidth', border_w, 'HandleVisibility', 'off'); 
    end
end
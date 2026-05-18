function plot_4_unicycle_trajectories(traj1, traj2, traj3, traj4, legend_labels, plot_title, fig_number)
    % PLOT_4_UNICYCLE_TRAJECTORIES Plots trajectories with obstacles and zones.
    
    % --- 0. DEFINIZIONE VARIABILI DALLA TABELLA ---
    x1 = -2.90; x2 = -1.60; x3 = -1.50; x4 = -1.40; x5 = -0.60;
    x6 = 0.20;  x7 = 1.00;  x8 = 1.45; x9 = 1.80; x10 = 2.20; x11 = 2.90;
    y1 = -1.50; y2 = -1.10; y3 = -0.70; y4 = -0.30; y5 = -0.20;
    y6 = 0.10;  y7 = 0.20;  y8 = 1.50;
    
    % Verifica dimensioni traiettorie
    trajs = {traj1, traj2, traj3, traj4};
    for i = 1:4
        if size(trajs{i}, 1) ~= 3
            error('Errore: La traiettoria %d non ha 3 righe (x, y, theta).', i);
        end
    end
    
    % Setup figura
    fig = figure(fig_number); 
    clf(fig);
    set(fig, 'Name', 'Environment Analysis', 'Color', 'w');
    hold on;
    
    % --- 1. DISEGNA LE ZONE COLORATE ---
    % Zona Blu (INITIAL POSITION)
    patch([x1 x3 x3 x1], [y6 y6 y8 y8], [0.75 0.85 0.95], 'EdgeColor', 'none', 'FaceAlpha', 0.6, 'DisplayName', 'Initial Position');
    text(x1+0.2, (y6+y8)/2, 'INITIAL POSITION', 'FontWeight', 'bold', 'FontSize', 10);
    
    % Zona Rossa (Corridoio)
    patch([x5 x6 x6 x5], [y1 y1 y6 y6], [1.0 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
    
    % Zona Arancione (Target Area)
    patch([x7 x11 x11 x7], [y1 y1 y8 y8], [0.9 0.6 0.4], 'EdgeColor', 'none', 'FaceAlpha', 0.4, 'HandleVisibility', 'off');
    
    % --- 2. DISEGNA GLI OSTACOLI ---
    obs_gray = [0.5 0.5 0.5];
    obs_brown = [0.5 0.3 0.2];
    
    % Blocco 1 (Grigio - Sinistra)
    patch([x4 x5 x5 x4], [y3 y3 y6 y6], obs_gray, 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Ostacoli');
    
    % Blocco 2 (Grigio - Destra)
    patch([x6 x7 x7 x6], [y1 y1 y3 y3], obs_gray, 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Blocco 3 (Marrone - Target Area)
    patch([x9 x10 x10 x9], [y5 y5 y7 y7], obs_brown, 'FaceAlpha', 0.9, 'EdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % --- 3. DISEGNA LE LINEE NERE (Confini) ---
    % Linea da (-1.4, -1.5) fino al secondo ostacolo (x6)
    plot([x4 x6], [y1 y1], 'k', 'LineWidth', 2.5, 'HandleVisibility', 'off');
    
    % Linea da (1, 0.1) fino al primo ostacolo (x5)
    plot([x7 x5], [y6 y6], 'k', 'LineWidth', 2.5, 'HandleVisibility', 'off');
    
    % --- 4. DISEGNA TRAIETTORIE E SIMBOLI ---
    colors = {'r', 'b', 'g', 'm'};
    for i = 1:4
        plot(trajs{i}(1, :), trajs{i}(2, :), 'Color', colors{i}, 'LineWidth', 1.8, 'DisplayName', legend_labels{i});
    end
    
    % --- NUOVO: CALCOLO DIMENSIONE REALE ROBOT ---
    % Diametro 20.5 cm = 0.205 m. Il raggio sarà 0.1025 m.
    robot_radius = 0.205 / 2;
    num_symbols_target = 6;
    
    for i = 1:4
        plot_trajectory_symbols(trajs{i}(1,:), trajs{i}(2,:), trajs{i}(3,:), ...
                                robot_radius, num_symbols_target, colors{i}, 0.5, 0.5);
    end
    
    % Grafica finale
    axis equal; grid on;
    xlim([x1-0.2, x11+0.2]); ylim([y1-0.2, y8+0.2]);
    xlabel('X [m]', 'FontWeight', 'bold'); ylabel('Y [m]', 'FontWeight', 'bold');
    title(plot_title, 'FontSize', 14);
    legend('Location', 'northeastoutside');
    hold off;
end

function plot_trajectory_symbols(x, y, theta, radius, num_target, color, alpha, border_w)
    N = length(x);
    if N < 2, return; end
    step = max(1, round(N / num_target));
    indices = unique([1, step:step:N, N]);
    
    % Pre-calcola un array di angoli per disegnare il cerchio
    th_circle = linspace(0, 2*pi, 30);
    
    for i = indices
        xi = x(i); yi = y(i); th_i = theta(i);
        
        % 1. Disegna il cerchio
        cx = radius * cos(th_circle) + xi;
        cy = radius * sin(th_circle) + yi;
        
        patch(cx, cy, color, 'FaceAlpha', alpha, 'EdgeColor', color, ...
              'LineWidth', border_w, 'HandleVisibility', 'off'); 
              
        % 2. Disegna l'orientazione (segmento dal centro al bordo)
        edge_x = xi + radius * cos(th_i);
        edge_y = yi + radius * sin(th_i);
        
        % Utilizzo 'k' (nero) per far risaltare visivamente la direzione sul cerchio colorato
        plot([xi, edge_x], [yi, edge_y], 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
end
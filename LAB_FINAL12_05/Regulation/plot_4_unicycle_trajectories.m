function plot_4_unicycle_trajectories(traj1, traj2, traj3, traj4, legend_labels, plot_title)
    % PLOT_4_UNICYCLE_TRAJECTORIES Plots 4 different trajectories with unicycle symbols.
    %
    % Input:
    %   traj1, traj2, traj3, traj4 - Matrici 3xN (x, y, theta) delle traiettorie
    %   legend_labels - Cell array con i 4 nomi per la legenda (es. {'A', 'B', 'C', 'D'})
    %   plot_title    - Stringa per il titolo principale del grafico

    % Verifica dimensioni
    trajs = {traj1, traj2, traj3, traj4};
    for i = 1:4
        if size(trajs{i}, 1) ~= 3
            error('Errore: La traiettoria %d non ha 3 righe (x, y, theta).', i);
        end
    end

    % Colori associati alle 4 traiettorie (Rosso, Blu, Verde, Magenta)
    colors = {'r', 'b', 'g', 'm'};
    
    % Stili di linea (puoi modificarli se vuoi distinguere ideal/actual, es. '--')
    line_styles = {'-', '-', '-', '-'}; 

    % Setup della figura

    %figure('Name', 'Unicycle Trajectories Comparison', 'Color', 'w');
    %hold on;
    % Setup della figura 1 (Traiettorie)
    fig = figure(1); 
    clf(fig); % Pulisce i dati dell'esecuzione precedente
    set(fig, 'Name', 'Unicycle Trajectories Comparison', 'Color', 'w');
    hold on;

    
    % 1. Disegna prima le linee delle traiettorie
    for i = 1:4
        plot(trajs{i}(1, :), trajs{i}(2, :), ...
             'Color', colors{i}, 'LineStyle', line_styles{i}, ...
             'LineWidth', 1.5, 'DisplayName', legend_labels{i});
    end
    
    % --- DYNAMIC SIZING FOR UNICYCLE SYMBOLS ---
    % Concatena tutte le X e tutte le Y per trovare l'estensione totale del grafico
    all_x = [traj1(1,:), traj2(1,:), traj3(1,:), traj4(1,:)];
    all_y = [traj1(2,:), traj2(2,:), traj3(2,:), traj4(2,:)];
    
    max_x = max(all_x); min_x = min(all_x);
    max_y = max(all_y); min_y = min(all_y);
    span = max(max_x - min_x, max_y - min_y);
    
    if span == 0
        span = 1; % Safety fallback
    end
    
    % Dimensione dinamica al 4% dello span totale
    base_L = span * 0.04; 
    
    % Vertici di base del triangolo
    base_vertices = [base_L, 0; -0.3*base_L, 0.4*base_L; -0.3*base_L, -0.4*base_L];

    % Configurazione per i simboli
    num_symbols_target = 12;   % Simboli per ogni traiettoria
    symbol_opacity = 0.5;      % Trasparenza
    symbol_border_width = 0.5; % Spessore bordo

    % 2. Disegna i simboli (triangolini) per tutte e 4 le traiettorie
    for i = 1:4
        plot_trajectory_symbols(trajs{i}(1,:), trajs{i}(2,:), trajs{i}(3,:), ...
                                base_vertices, num_symbols_target, colors{i}, ...
                                symbol_opacity, symbol_border_width);
    end

    % Impostazioni grafiche
    axis equal; 
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    title(plot_title);
    legend('Location', 'best');
    
    hold off;
end

% --- Helper Function to Plot Symbols ---
function plot_trajectory_symbols(x, y, theta, base_v, num_target, color, alpha, border_w)
    N = length(x);
    if N < 2, return; end % Nothing to plot
    
    % Calculate sampling interval
    step = max(1, round(N / num_target));
    indices = 1:step:N;
    
    for i = indices
        % Get current state
        xi = x(i); yi = y(i); th_i = theta(i);
        
        % Create rotation matrix
        R = [cos(th_i), -sin(th_i); sin(th_i), cos(th_i)];
        
        % Rotate and translate base vertices
        rotated_v = (R * base_v')';
        translated_v = rotated_v + [xi, yi];
        
        % Plot the triangle patch
        patch('Vertices', translated_v, 'Faces', [1 2 3], ...
              'FaceColor', color, 'FaceAlpha', alpha, ...
              'EdgeColor', color, 'LineWidth', border_w, ...
              'HandleVisibility', 'off'); 
    end
end
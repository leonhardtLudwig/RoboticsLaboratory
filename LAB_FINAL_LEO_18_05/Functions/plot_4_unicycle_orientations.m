function plot_4_unicycle_orientations(traj1, traj2, traj3, traj4, legend_labels, plot_title)
    % PLOT_4_UNICYCLE_ORIENTATIONS Plotta l'andamento dell'orientazione (theta) 
    % per 4 traiettorie al fine di valutare oscillazioni ed errore.
    %
    % Input:
    %   traj1, traj2, traj3, traj4 - Matrici 3xN (x, y, theta)
    %   legend_labels - Cell array con i 4 nomi per la legenda
    %   plot_title    - Stringa per il titolo principale del grafico

    % Verifica dimensioni
    trajs = {traj1, traj2, traj3, traj4};
    for i = 1:4
        if size(trajs{i}, 1) ~= 3
            error('Errore: La traiettoria %d non ha 3 righe (x, y, theta).', i);
        end
    end

    % Colori e stili (coerenti con la funzione della traiettoria 2D)
    colors = {'r', 'b', 'g', 'm'};
    line_styles = {'-', '-', '-', '-'}; 

    % Setup della figura
    figure('Name', 'Orientations Comparison', 'Color', 'w');
    hold on;
    
    % Estrai e plotta la terza riga (theta) per ciascuna traiettoria
    for i = 1:4
        theta = trajs{i}(3, :);
        N = length(theta);
        campioni = 1:N; % Asse X: indice del campione
        
        plot(campioni, theta, ...
             'Color', colors{i}, 'LineStyle', line_styles{i}, ...
             'LineWidth', 1.5, 'DisplayName', legend_labels{i});
    end
    
    % Impostazioni grafiche
    grid on;
    xlabel('Campioni (Samples)');
    ylabel('Orientazione \theta [rad]');
    title(plot_title);
    legend('Location', 'best');
    
    hold off;
end
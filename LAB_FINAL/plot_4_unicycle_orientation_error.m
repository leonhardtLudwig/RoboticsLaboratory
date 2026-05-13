function plot_4_unicycle_orientation_error(traj1, traj2, traj3, traj4, legend_labels, plot_title, fig_number)
    % PLOT_4_UNICYCLE_ORIENTATION_ERROR Plotta l'errore di orientazione 
    % (con angle wrapping) per 4 traiettorie rispetto al tempo.
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
    
    % Colori e stili
    colors = {'r', 'b', 'g', 'm'};
    line_styles = {'-', '-', '-', '-'}; 
    
    % Setup della figura 2 (Errore di orientazione)
    fig = figure(fig_number); 
    clf(fig); % Pulisce i dati dell'esecuzione precedente
    set(fig, 'Name', 'Orientation Error Comparison', 'Color', 'w');
    hold on;
    
    % --- DEFINIZIONE TEMPO DI CAMPIONAMENTO ---
    T_s = 0.04; % [s]
    
    % Estrai, wrappa e plotta theta per ciascuna traiettoria
    for i = 1:4
        theta_raw = trajs{i}(3, :);
        
        % --- ANGLE WRAPPING ---
        % Essendo l'angolo desiderato = 0, l'errore è theta_raw.
        % Questa operazione forza l'angolo (errore) nell'intervallo [-pi, pi]
        theta_error_wrapped = atan2(sin(theta_raw), cos(theta_raw));
        
        N = length(theta_error_wrapped);
        
        % --- ASSE DEL TEMPO ---
        % Crea un vettore temporale che va da 0 a (N-1)*T_s
        tempo = (0:N-1) * T_s; 
        
        plot(tempo, theta_error_wrapped, ...
             'Color', colors{i}, 'LineStyle', line_styles{i}, ...
             'LineWidth', 1.5, 'DisplayName', legend_labels{i});
    end
    
    % --- LINEA DI RIFERIMENTO (TARGET) ---
    % Aggiunge una linea tratteggiata nera sullo 0 per visualizzare chiaramente la convergenza
    yline(0, 'k--', 'Target (\theta_e = 0)', 'LineWidth', 1, 'HandleVisibility', 'off');
    
    % Impostazioni grafiche
    grid on;
    
    % Modifica della Label X
    xlabel('Tempo [s]');
    ylabel('Errore di Orientazione \theta_e [rad]');
    title(plot_title);
    
    % Imposta i limiti dell'asse Y leggermente oltre [-pi, pi] per una migliore visualizzazione
    ylim([-pi - 0.5, pi + 0.5]);
    
    % Opzionale: puoi limitare l'asse X usando la lunghezza massima del tempo
    % xlim([0, max(tempo)]);
    
    % Aggiunge tick marks speciali per Pi greco sull'asse Y
    yticks([-pi, -pi/2, 0, pi/2, pi]);
    yticklabels({'-\pi', '-\pi/2', '0', '\pi/2', '\pi'});
    
    legend('Location', 'best');
    
    hold off;
end
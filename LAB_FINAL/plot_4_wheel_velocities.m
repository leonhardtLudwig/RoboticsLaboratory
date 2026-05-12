function plot_4_wheel_velocities(vw1, vw2, vw3, vw4, legend_labels, plot_title)
    % PLOT_4_WHEEL_VELOCITIES Plotta le velocità delle due ruote per 4 traiettorie.
    %
    % Input:
    %   vw1, vw2, vw3, vw4 - Matrici Nx2 (Velocità Ruota 1, Velocità Ruota 2)
    %   legend_labels      - Cell array con i 4 nomi per la legenda
    %   plot_title         - Stringa per il titolo principale della figura

    % Raggruppa in un cell array
    v_wheels = {vw1, vw2, vw3, vw4};
    
    % Controllo e correzione dimensioni (assicura che siano Nx2 e non 2xN)
    for i = 1:4
        if size(v_wheels{i}, 2) ~= 2
            if size(v_wheels{i}, 1) == 2
                v_wheels{i} = v_wheels{i}'; % Traspone se l'utente passa 2xN
            else
                error('Errore: La matrice delle velocità %d deve essere Nx2.', i);
            end
        end
    end

    % Colori e stili (coerenti con le altre figure)
    colors = {'r', 'b', 'g', 'm'};
    line_styles = {'-', '-', '-', '-'}; 
    
    % Setup della figura 3 (Velocità Ruote)
    fig = figure(3); 
    clf(fig); % Pulisce i dati dell'esecuzione precedente
    set(fig, 'Name', 'Wheel Velocities Comparison', 'Color', 'w');
    
    % Titolo globale sopra i subplot
    sgtitle(plot_title, 'FontWeight', 'bold', 'FontSize', 12);
    
    % --- DEFINIZIONE TEMPO DI CAMPIONAMENTO ---
    T_s = 0.04; % [s]
    
    % ==========================================
    % SUBPLOT 1: Velocità Ruota 1 (Colonna 1)
    % ==========================================
    subplot(2, 1, 1);
    hold on;
    for i = 1:4
        N = size(v_wheels{i}, 1);
        tempo = (0:N-1) * T_s; 
        
        plot(tempo, v_wheels{i}(:, 1), ...
             'Color', colors{i}, 'LineStyle', line_styles{i}, ...
             'LineWidth', 1.5, 'DisplayName', legend_labels{i});
    end
    grid on;
    ylabel('Velocità Ruota 1 [rad/s]'); % Modifica unità di misura se necessario (es. m/s)
    title('Motore 1 (es. Destro)');
    legend('Location', 'best');
    hold off;

    % ==========================================
    % SUBPLOT 2: Velocità Ruota 2 (Colonna 2)
    % ==========================================
    subplot(2, 1, 2);
    hold on;
    for i = 1:4
        N = size(v_wheels{i}, 1);
        tempo = (0:N-1) * T_s; 
        
        plot(tempo, v_wheels{i}(:, 2), ...
             'Color', colors{i}, 'LineStyle', line_styles{i}, ...
             'LineWidth', 1.5, 'DisplayName', legend_labels{i});
    end
    grid on;
    xlabel('Tempo [s]');
    ylabel('Velocità Ruota 2 [rad/s]'); % Modifica unità di misura se necessario (es. m/s)
    title('Motore 2 (es. Sinistro)');
    hold off;
    
end
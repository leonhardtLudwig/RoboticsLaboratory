function analyze_EKF_results(z_EKF, P_EKF, z_true, fig_number)
    % ANALYZE_EKF_RESULTS Plotta gli stati dell'EKF con i limiti a 3-sigma 
    % e calcola l'errore rispetto alla simulazione offline.

    % Costanti dal setup del sistema
    T_s = 0.04; 
    num_states = 7;
    N = size(z_EKF, 2);
    
    % Nomi esatti basati sul vettore z_k fornito
    state_names = {'x [m]', 'y [m]', '\theta [rad]', ...
                   '\delta\phi_{L} [rad]', '\delta\phi_{R} [rad]', ...
                   '\omega_{L} [rad/s]', '\omega_{R} [rad/s]'};
                   
    % CORREZIONE 1: Controlla che gli argomenti siano almeno 3
    has_truth = (nargin >= 3) && ~isempty(z_true);
    
    % Asse temporale
    tempo = (0:N-1) * T_s;
    
    % Setup Figura
    fig = figure(fig_number);
    clf(fig);
    set(fig, 'Name', 'Analisi EKF e Limiti di Confidenza', 'Color', 'w');
    fig.Position = [100, 100, 1200, 800]; 
    sgtitle('Analisi Filtro di Kalman: Stati vs \pm3\sigma vs Offline', 'FontWeight', 'bold', 'FontSize', 14);
    
    % --- INIZIO OUTPUT TERMINALE ---
    fprintf('\n======================================================\n');
    fprintf('               ANALISI FILTRO DI KALMAN               \n');
    fprintf('======================================================\n');
    
    for i = 1:num_states
        subplot(4, 2, i);
        hold on; grid on;
        
        stima = z_EKF(i, :);
        varianza = squeeze(P_EKF(i, i, :))'; 
        sigma = sqrt(varianza);
        
        % Limiti di confidenza 99.7%
        upper_bound = stima + 3 * sigma;
        lower_bound = stima - 3 * sigma;
        
        % Plot Limiti e Stima EKF
        plot(tempo, upper_bound, 'r--', 'LineWidth', 1, 'DisplayName', 'Limiti \pm3\sigma');
        plot(tempo, lower_bound, 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');
        plot(tempo, stima, 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF (Online)');
        
        mean_sigma = mean(sigma);
        final_sigma = sigma(end);
        rmse_str = 'N/A';
        
        % CORREZIONE 2: Verifica che la riga 'i' esista nella ground truth
        if has_truth && i <= size(z_true, 1)
            vero = z_true(i, :);
            
            if i == 3
                errore = atan2(sin(stima - vero), cos(stima - vero));
            else
                errore = stima - vero;
            end
            
            plot(tempo, vero, 'k-', 'LineWidth', 1.2, 'DisplayName', 'Truth (Offline)');
            
            rmse = sqrt(mean(errore.^2));
            rmse_str = sprintf('%.4f', rmse);
        end
        
        % Impostazioni grafiche
        title(state_names{i}, 'FontWeight', 'bold');
        xlabel('Tempo [s]');
        
        if i == 1
            legend('Location', 'best');
        end
        hold off;
        
        fprintf('Stato %d [%-15s] | RMSE: %-7s | Sigma Medio: %.4f | Sigma Finale: %.4f\n', ...
                i, state_names{i}, rmse_str, mean_sigma, final_sigma);
    end
    
    fprintf('======================================================\n\n');
end
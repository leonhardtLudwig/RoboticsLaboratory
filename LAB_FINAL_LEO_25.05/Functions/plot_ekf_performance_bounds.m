function plot_ekf_performance_bounds(exp, exp_title, fig_num)
    % PLOT_EKF_PERFORMANCE_BOUNDS Plotta l'andamento temporale degli errori di stima
    % (colonna sx) con intervalli di confidenza a +/- 3-sigma (Fill) e 
    % l'andamento delle deviazioni standard (colonna dx). Replicando image_9.png.
    
    fig = figure(fig_num);
    clf(fig);
    set(fig, 'Name', 'EKF XYZ Estimation and Bounds', 'Color', 'w');
    
    % Titolo principale della dashboard
    sgtitle(sprintf('EKF Performance Analysis (XYZ) - %s', exp_title), ...
            'Interpreter', 'none', 'FontWeight', 'bold', 'FontSize', 14);
    
    Ts = 0.04;
    N = size(exp.q_EKF, 2);
    t = (0:N-1) * Ts;
    
    % --- 1. ESTRAZIONE DEVIAZIONE STANDARD ---
    % Calcoliamo sigma = sqrt(P_ii) per gli stati di posa [x, y, theta]
    % sigma sarà 3xN
    sigma_xyz = zeros(3, N);
    for i = 1:3
        sigma_xyz(i, :) = sqrt(squeeze(exp.P_filt_EKF(i, i, :)))';
    end
    
    % --- 2. CONFIGURAZIONE PLOT ---
    % Etichette per i subplot
    states = {'X', 'Y', '\theta'};
    units = {'[m]', '[m]', '[rad]'};
    
    % Definizione di colori moderni (per Mean Error e Sigma)
    colors = lines(3); % Colori standard XYZ
    t_fill = [t, fliplr(t)];
    
    % --- 3. CREAZIONE GRAFICI ---
    for state_idx = 1:3
        current_color = colors(state_idx, :);
        sig = sigma_xyz(state_idx, :);
        
        % =====================================================================
        % COLONNA SINISTRA: ERRORE DI STIMA CON BANDE DI CONFIDENZA
        % =====================================================================
        subplot(3, 2, (state_idx*2) - 1);
        hold on; grid on;
        
        % IL FIX È QUI: Dati dell'errore di STIMA (Ground Truth vs EKF). 
        % gt_real_gap è Nx3, prendiamo la colonna e la trasponiamo per avere 1xN
        mean_err = exp.gt_real_gap(:, state_idx)';
        
        % Calcolo intervallo di confidenza a +/- 3-sigma (rispetto allo zero)
        upper_bound = 3 * sig;
        lower_bound = -3 * sig;
        
        % Costruzione dell'area ombreggiata (rispetto allo zero)
        fill_data = [upper_bound, fliplr(lower_bound)];
        
        % 1. Disegna l'area ombreggiata (trasparente al 20%)
        fill(t_fill, fill_data, current_color, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        
        % 2. Disegna i bordi esterni tratteggiati (+/- 3-sigma)
        plot(t, upper_bound, '--', 'Color', current_color, 'LineWidth', 0.5, 'HandleVisibility', 'off');
        plot(t, lower_bound, '--', 'Color', current_color, 'LineWidth', 0.5, 'DisplayName', '\pm 3\sigma Confidence Bounds');
        
        % 3. Disegna il valore medio dell'errore (linea solida)
        plot(t, mean_err, 'Color', current_color, 'LineWidth', 1.8, 'DisplayName', 'Estimation Error (GT vs EKF)');
        
        % 4. Disegna linea orizzontale a zero per riferimento
        plot([0 t(end)], [0 0], 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
        
        % Titolo e labels
        title(sprintf('%s-Estimation Error & +/- 3\\sigma Bounds', states{state_idx}));
        ylabel(sprintf('Est. Error %s %s', states{state_idx}, units{state_idx}), 'FontWeight', 'bold');
        xlim([0, t(end)]);
        
        % Ottimizzazione limiti asse Y per non avere plot "schiacciati"
        % Usiamo il massimo tra l'errore reale e i bounds per scalare bene il grafico
        max_y_val = max(max(abs(mean_err)), max(upper_bound));
        if max_y_val > 0
            ylim([-max_y_val*1.2, max_y_val*1.2]);
        end
        
        legend('Location', 'best');
        
        % Solo l'ultimo subplot ha l'xlabel
        if state_idx == 3
            xlabel('Time [s]');
        end
        % =====================================================================
        % COLONNA DESTRA: DEVIAZIONE STANDARD (SIGMA)
        % =====================================================================
        subplot(3, 2, state_idx*2);
        hold on; grid on;
        
        % Plotta la deviazione standard grezza
        plot(t, sig, 'Color', current_color, 'LineWidth', 1.5, 'DisplayName', '\sigma');
        
        % Titolo e labels
        title(sprintf('\\sigma_%s over Time', states{state_idx}));
        ylabel(sprintf('\\sigma_%s %s', states{state_idx}, units{state_idx}), 'FontWeight', 'bold');
        xlim([0, t(end)]);
        grid on;
        legend('Location', 'best');
        
        % Solo l'ultimo subplot ha l'xlabel
        if state_idx == 3
            xlabel('Time [s]');
        end
    end
end
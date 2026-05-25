function plot_exp_errors(exp, exp_title, fig_num, task_type)
    % PLOT_EXP_ERRORS Plotta gli errori di stato rispetto al riferimento.
    %
    % Input:
    %   exp        - Struct dell'esperimento
    %   exp_title  - Titolo del grafico
    %   fig_num    - Numero della figura
    %   task_type  - Stringa: 'regulation' oppure 'tracking'
    
    % Seleziona o crea la figura e la pulisce
    figure(fig_num);
    clf(fig_num);
    
    % --- LOGICA TITOLO ---
    if strcmpi(task_type, 'tracking')
        main_title = 'Tracking Errors';
    else
        main_title = 'Regulation Errors';
    end
    
    sgtitle(sprintf('%s - %s', main_title, exp_title), 'Interpreter', 'none', 'FontWeight', 'bold');
    
    % Etichette per i subplot
    labels = {'Error X [m]', 'Error Y [m]', 'Error \theta [rad]'};
    
    % Creazione del vettore dei tempi
    Ts = 0.04; 
    N = size(exp.err_model_des, 2); % Essendo 3xN, il numero di campioni è il numero di colonne
    t = (0:N-1) * Ts;
    
    for i = 1:3
        subplot(3, 1, i);
        hold on; grid on;
        
        % Plot rispetto al vettore dei tempi 't'
        plot(t, exp.err_model_des(i, :), 'LineWidth', 1.5, 'DisplayName', 'Model vs Des');
        plot(t, exp.err_gt_des(i, :), 'LineWidth', 1.5, 'DisplayName', 'GT vs Des');
        
        % Plotta EKF se presente
        if isfield(exp, 'err_ekf_des')
            plot(t, exp.err_ekf_des(i, :), '--', 'LineWidth', 1.5, 'DisplayName', 'EKF vs Des');
        end
        
        ylabel(labels{i});
        
        % Limita l'asse X esattamente dall'inizio alla fine dei dati
        xlim([0, t(end)]);
        
        if i == 1
            legend('Location', 'best');
        end
        
        if i == 3
            xlabel('Time [s]');
        end
    end
end
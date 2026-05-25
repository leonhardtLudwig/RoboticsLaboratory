function plot_gaps(exp, exp_title, fig_num)
    % Seleziona o crea la figura e la pulisce
    figure(fig_num);
    clf(fig_num);
    
    sgtitle(sprintf('Reality Gaps - %s', exp_title), 'Interpreter', 'none');
    labels = {'Gap X [m]', 'Gap Y [m]', 'Gap \theta [rad]'};
    
    % Creazione del vettore dei tempi
    Ts = 0.04;
    N = size(exp.sim_real_gap, 1); % Essendo Nx3, il numero di campioni è il numero di righe
    t = (0:N-1) * Ts;
    
    for i = 1:3
        subplot(3, 1, i);
        hold on; grid on;
        
        % Plot rispetto al vettore dei tempi 't'
        plot(t, exp.sim_real_gap(:, i), 'LineWidth', 1.5, 'DisplayName', 'Sim - Real Gap');
        plot(t, exp.gt_real_gap(:, i), 'LineWidth', 1.5, 'DisplayName', 'GT - Real Gap');
        
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
function plot_EKF_covariance_evolution(P_EKF, fig_number)
    % PLOT_EKF_COVARIANCE_EVOLUTION Traccia l'andamento nel tempo delle 
    % incertezze (deviazioni standard sigma) per i 7 stati dell'EKF.
    %
    % Input:
    %   P_EKF      - Matrice 7x7xN (Covarianze stimate)
    %   fig_number - (Opzionale) ID della figura, default 6

    

    % Costanti dal setup del sistema
    T_s = 0.04; 
    num_states = 7;
    N = size(P_EKF, 3);
    
    % Nomi degli stati
    state_names = {'x [m]', 'y [m]', '\theta [rad]', ...
                   '\delta\phi_{L} [rad]', '\delta\phi_{R} [rad]', ...
                   '\omega_{L} [rad/s]', '\omega_{R} [rad/s]'};
               
    % Asse temporale
    tempo = (0:N-1) * T_s;
    
    % Setup Figura
    fig = figure(fig_number);
    clf(fig);
    set(fig, 'Name', 'Evoluzione Incertezza EKF', 'Color', 'w');
    fig.Position = [150, 150, 1200, 800]; 
    sgtitle('Evoluzione dell''Incertezza EKF (Deviazione Standard \sigma)', 'FontWeight', 'bold', 'FontSize', 14);
    
    for i = 1:num_states
        subplot(4, 2, i);
        hold on; grid on;
        
        % Estrae la varianza (elemento i,i per tutti gli istanti di tempo)
        varianza = squeeze(P_EKF(i, i, :))'; 
        
        % Calcola la deviazione standard
        sigma = sqrt(varianza);
        
        % Plotta l'evoluzione di sigma
        plot(tempo, sigma, 'b-', 'LineWidth', 1.5);
        
        % Impostazioni grafiche
        title(['Incertezza su ', state_names{i}], 'FontWeight', 'bold');
        xlabel('Tempo [s]');
        ylabel('\sigma');
        
        % Imposta il limite Y minimo a zero (l'incertezza non può essere negativa)
        ylim_current = ylim;
        ylim([0, ylim_current(2) * 1.1]); % Lascia un 10% di margine in alto
        
        hold off;
    end
end